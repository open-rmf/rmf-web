import asyncio
from dataclasses import dataclass
from re import Match
from typing import Any, Callable, Coroutine, TypedDict, cast
from urllib.parse import unquote as url_unquote

import pydantic
import socketio
from fastapi import APIRouter, FastAPI
from fastapi.exceptions import HTTPException
from fastapi.requests import HTTPConnection
from fastapi.routing import APIRoute
from reactivex import Observable
from reactivex.abc import DisposableBase
from starlette.routing import compile_path

from api_server.logging import LoggerAdapter, get_logger
from api_server.models.user import User

from .errors import *
from .pydantic_json_serializer import PydanticJsonSerializer
from .singleton_dep import SingletonDep, singleton_dep


class SioSession(TypedDict):
    environ: dict
    user: User
    subscriptions: dict[str, DisposableBase]
    logger: LoggerAdapter


@dataclass
class SubscriptionRequest:
    sid: str
    sio: socketio.AsyncServer
    room: str
    user: User
    environ: dict
    logger: LoggerAdapter


@dataclass
class SubscriptionResponse:
    success: bool
    error: str = ""


@dataclass
class SubscriptionData:
    room: str


class SubRoute:
    def __init__(
        self,
        path: str,
        endpoint: Callable[
            [SubscriptionRequest], Observable | Coroutine[Any, Any, Observable]
        ],
        *,
        response_model: type[pydantic.BaseModel] | None = None,
    ):
        self.path = path
        self.endpoint = endpoint
        self.path_regex, self.path_format, self.param_convertors = compile_path(path)
        self.response_model = response_model

    def matches(self, path: str) -> Match | None:
        return self.path_regex.match(path)


OnSubscribe = Callable[..., Observable | Coroutine[Any, Any, Observable]]


class FastIORouter(APIRouter):
    """
    This is a router that extends on the default fastapi router. It adds a new
    method "sub" which registers a socket.io endpoint.

    .. code-block::
        import uvicorn
        from rx import operators as rxops
        from rx.subject.subject import Subject

        from api_server.fast_io import FastIORouter, SubscriptionRequest

        app = FastIO()
        router = FastIORouter()
        obs = Subject()


        @router.sub("/video_rental/{film}/available")
        def router_sub_availability(req: SubscriptionRequest, film: str):
            return obs.pipe(
                rxops.filter(lambda x: x["film"] == film)
            )


        @router.post("/video_rental/{film}/return")
        def router_post_return_video(film: str):
            obs.on_next({"film": film})


        app.include_router(router)

        uvicorn.run(app)
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.sub_routes: list[SubRoute] = []

    def include_router(self, router: APIRouter, **kwargs):
        super().include_router(router, **kwargs)

        if isinstance(router, FastIORouter):
            prefix = kwargs.get("prefix", "")

            for r in router.sub_routes:
                self.sub_routes.append(
                    SubRoute(
                        prefix + router.prefix + r.path,
                        r.endpoint,
                    )
                )

    def sub(self, path: str, *, response_model: type[pydantic.BaseModel] | None = None):
        """
        Registers a socket.io endpoint which handles subscriptions.
        """

        def decorator(func: OnSubscribe) -> OnSubscribe:
            self.sub_routes.append(SubRoute(path, func, response_model=response_model))
            return func

        return decorator


class FastIO(FastAPI):
    def __init__(
        self,
        *args,
        socketio_path: str = "/socket.io",
        socketio_connect: (
            Callable[[str, dict, dict | None], Coroutine[Any, Any, User | None]] | None
        ) = None,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self._socketio_connect = socketio_connect

        self.sio = socketio.AsyncServer(
            async_mode="asgi", cors_allowed_origins=[], json=PydanticJsonSerializer()
        )
        self.sio.on("connect", self._on_connect)
        self.sio.on("subscribe", self._on_subscribe)
        self.sio.on("unsubscribe", self._on_unsubscribe)
        self.sio.on("disconnect", self._on_disconnect)
        self._sub_routes: list[SubRoute] = []

        self._sio_route = APIRoute(
            socketio_path,
            lambda: None,
            summary="Socket.io endpoint",
            description="""
# NOTE: This endpoint is here for documentation purposes only, this is _not_ a REST endpoint.

## About
This exposes a minimal pubsub system built on top of socket.io.
It works similar to a normal socket.io endpoint, except that are 2 special
rooms which control subscriptions.

## Rooms
### subscribe
Clients must send a message to this room to start receiving messages on other rooms.
The message must be of the form:

```
{
    "room": "<room_name>"
}
```

### unsubscribe
Clients can send a message to this room to stop receiving messages on other rooms.
The message must be of the form:

```
{
    "room": "<room_name>"
}
```
            """,
        )
        self.routes.append(self._sio_route)

        self._sio_app = socketio.ASGIApp(self.sio, socketio_path="")
        self.mount(socketio_path, self._sio_app)

    def include_router(self, router: APIRouter, *args, prefix: str = "", **kwargs):
        super().include_router(router, *args, prefix=prefix, **kwargs)
        room_descriptions = ""

        if isinstance(router, FastIORouter):
            for r in router.sub_routes:
                full_path = prefix + router.prefix + r.path
                self._sub_routes.append(
                    SubRoute(
                        full_path,
                        r.endpoint,
                        response_model=r.response_model,
                    )
                )
                if r.response_model:
                    response_schema = f"""
```
{r.response_model.schema_json(indent=2)}
```
"""
                else:
                    response_schema = ""
                docstring = r.endpoint.__doc__ or ""
                room_descriptions += f"""
### {full_path}
{docstring.strip()}
{response_schema}
"""
            self._sio_route.description += room_descriptions

    @staticmethod
    def _parse_sub_data(data: dict) -> SubscriptionData:
        if "room" not in data:
            raise SubscribeError("missing 'room'")
        room = url_unquote(data["room"])
        return SubscriptionData(room=room)

    def _match_routes(
        self, sub_data: SubscriptionData
    ) -> tuple[Match, SubRoute] | None:
        for r in self._sub_routes:
            match = r.matches(sub_data.room)
            if match:
                return match, r
        return None

    async def _on_connect(self, sid: str, environ: dict, auth: dict | None = None):
        logger = get_logger(HTTPConnection(environ))
        user = (
            await self._socketio_connect(sid, environ, auth)
            if self._socketio_connect
            else None
        )
        if user is None:
            return False

        async with self.sio.session(sid) as session:
            session: SioSession
            session["environ"] = environ
            session["user"] = user
            session["subscriptions"] = {}
            session["logger"] = logger

        return True

    async def _on_subscribe(self, sid: str, data: dict):
        try:
            sub_data = self._parse_sub_data(data)
        except SubscribeError as e:
            return SubscriptionResponse(success=False, error=str(e))

        try:
            result = self._match_routes(sub_data)
            if result is None:
                await self.sio.emit(
                    "subscribe", {"success": False, "error": "no events in path"}
                )
                return
            match, route = result

            session: SioSession = await self.sio.get_session(sid)
            req = SubscriptionRequest(
                sid=sid,
                sio=self.sio,
                room=sub_data.room,
                user=session["user"],
                environ=session["environ"],
                logger=session["logger"],
            )
            maybe_coro = route.endpoint(req, **match.groupdict())
            if asyncio.iscoroutine(maybe_coro):
                obs = await maybe_coro
            else:
                obs = maybe_coro
            obs = cast(Observable, obs)

            loop = asyncio.get_event_loop()

            def on_next(data):
                loop.create_task(self.sio.emit(sub_data.room, data, to=sid))

            sub = obs.subscribe(on_next)
            session.setdefault("subscriptions", {})[sub_data.room] = sub

        except HTTPException as e:
            return SubscriptionResponse(
                success=False, error=f"{e.status_code} {e.detail}"
            )

        return SubscriptionResponse(success=True)

    async def _on_unsubscribe(self, sid: str, data: dict):
        try:
            sub_data = self._parse_sub_data(data)
            async with self.sio.session(sid) as session:
                session: dict[Any, Any]
                sub = session["subscriptions"].get(sub_data.room)
                if sub is None:
                    raise SubscribeError("not subscribed to topic")
                sub.dispose()
                del session["subscriptions"][sub_data.room]
                await self.sio.emit("unsubscribe", {"success": True})
        except SubscribeError as e:
            await self.sio.emit("unsubscribe", {"success": False, "error": str(e)})

    async def _on_disconnect(self, sid: str):
        async with self.sio.session(sid) as session:
            subs = session.get("subscriptions")
            if subs is None:
                return
            for s in subs.values():
                s.dispose()
