import asyncio
from dataclasses import dataclass
from re import Match
from typing import (
    Any,
    Callable,
    Coroutine,
    Dict,
    Generic,
    List,
    Optional,
    Tuple,
    Type,
    TypeVar,
    Union,
    cast,
)
from urllib.parse import unquote as url_unquote

import pydantic
import socketio
from fastapi import APIRouter, FastAPI
from fastapi.exceptions import HTTPException
from fastapi.routing import APIRoute
from rx.core.observable.observable import Observable
from starlette.routing import compile_path

from .errors import *

SioServer = TypeVar("SioServer", bound=socketio.AsyncServer)


@dataclass
class SubscriptionRequest(Generic[SioServer]):
    sid: str
    sio: SioServer
    room: str
    session: Dict[Any, Any]


@dataclass
class SubscriptionData:
    room: str


class SubRoute:
    def __init__(
        self,
        path: str,
        endpoint: Callable[
            [SubscriptionRequest], Union[Observable, Coroutine[Any, Any, Observable]]
        ],
        *,
        response_model: Optional[Type[pydantic.BaseModel]] = None,
    ):
        self.path = path
        self.endpoint = endpoint
        self.path_regex, self.path_format, self.param_convertors = compile_path(path)
        self.response_model = response_model

    def matches(self, path: str) -> Optional[Match]:
        return self.path_regex.match(path)


OnSubscribe = Callable[..., Union[Observable, Coroutine[Any, Any, Observable]]]


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
        self.sub_routes: List[SubRoute] = []

    def include_router(self, router: "FastIORouter", **kwargs):
        super().include_router(router, **kwargs)
        prefix = kwargs.get("prefix", "")

        for r in router.sub_routes:
            self.sub_routes.append(
                SubRoute(
                    prefix + router.prefix + r.path,
                    r.endpoint,
                )
            )

    def sub(
        self, path: str, *, response_model: Optional[Type[pydantic.BaseModel]] = None
    ):
        """
        Registers a socket.io endpoint which handles subscriptions.
        """

        def decorator(func: OnSubscribe) -> OnSubscribe:
            self.sub_routes.append(SubRoute(path, func, response_model=response_model))
            return func

        return decorator


class FastIO(FastAPI, Generic[SioServer]):
    def __init__(
        self,
        sio: SioServer,
        *args,
        socketio_path: str = "/socket.io",
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.sio = sio
        self.sio.on("subscribe", self._on_subscribe)
        self.sio.on("unsubscribe", self._on_unsubscribe)
        self._sub_routes: List[SubRoute] = []

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

        self._sio_app = socketio.ASGIApp(
            self.sio, super().__call__, socketio_path=socketio_path
        )

    async def __call__(self, scope, receive, send):
        # Make the sio the "primary" app because fastapi does not support mounting a single path.
        return await self._sio_app(scope, receive, send)

    def include_router(self, router: APIRouter, prefix="", **kwargs):
        super().include_router(router, prefix=prefix, **kwargs)
        room_descriptions = ""

        if isinstance(router, FastIORouter):
            for r in router.sub_routes:
                self._sub_routes.append(
                    SubRoute(
                        prefix + router.prefix + r.path,
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
### {r.path}
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
    ) -> Optional[Tuple[Match, SubRoute]]:
        for r in self._sub_routes:
            match = r.matches(sub_data.room)
            if match:
                return match, r
        return None

    async def _on_subscribe(self, sid: str, data: dict):
        try:
            sub_data = self._parse_sub_data(data)
        except SubscribeError as e:
            await self.sio.emit("subscribe", {"success": False, "error": str(e)})
            return

        try:
            result = self._match_routes(sub_data)
            if result is None:
                await self.sio.emit(
                    "subscribe", {"success": False, "error": "no events in path"}
                )
                return
            match, route = result

            session: Dict[Any, Any] = await self.sio.get_session(sid)
            req = SubscriptionRequest(
                sid=sid, sio=self.sio, room=sub_data.room, session=session
            )
            maybe_coro = route.endpoint(req, **match.groupdict())
            if asyncio.iscoroutine(maybe_coro):
                obs = await maybe_coro
            else:
                obs = maybe_coro
            obs = cast(Observable, obs)

            loop = asyncio.get_event_loop()

            def on_next(data):
                if isinstance(data, pydantic.BaseModel):
                    data = data.dict()

                async def emit():
                    await self.sio.emit(sub_data.room, data, to=sid)

                loop.create_task(emit())

            sub = obs.subscribe(on_next)
            session.setdefault("_subscriptions", {})[sub_data.room] = sub

        except HTTPException as e:
            await self.sio.emit(
                "subscribe", {"success": False, "error": f"{e.status_code} {e.detail}"}
            )

        await self.sio.emit("subscribe", {"success": True}, sid)

    async def _on_unsubscribe(self, sid: str, data: dict):
        try:
            sub_data = self._parse_sub_data(data)
            async with self.sio.session(sid) as session:
                session: Dict[Any, Any]
                sub = session["_subscriptions"].get(sub_data.room)
                if sub is None:
                    raise SubscribeError("not subscribed to topic")
                sub.dispose()
                await self.sio.emit("unsubscribe", {"success": True})
        except SubscribeError as e:
            await self.sio.emit("unsubscribe", {"success": False, "error": str(e)})
