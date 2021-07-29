import asyncio
import inspect
import logging
from abc import ABC, abstractmethod
from copy import copy
from dataclasses import dataclass
from re import Pattern
from typing import (
    Any,
    Awaitable,
    Callable,
    Dict,
    Generic,
    List,
    Optional,
    Protocol,
    TypeVar,
    Union,
    cast,
)
from urllib.parse import unquote as url_unquote

import pydantic
import socketio
from fastapi import APIRouter, Depends, FastAPI, Path, Request
from fastapi.exceptions import HTTPException
from fastapi.types import DecoratedCallable
from rx import Observable
from starlette.routing import compile_path, replace_params

from api_server.authenticator import AuthenticationError, JwtAuthenticator
from api_server.fast_io.errors import *
from api_server.models import User

T = TypeVar("T")


class DataStore(ABC, Generic[T]):
    @abstractmethod
    async def get(
        self, key: str, path_params: Dict[str, str], user: User
    ) -> Union[T, None]:
        pass

    @abstractmethod
    async def set(self, key: str, path_params: Dict[str, str], data: T) -> None:
        pass


@dataclass
class SubscriptionData:
    path: str


@dataclass
class WatchRequest:
    sid: str
    sio: socketio.AsyncServer
    path: str
    user: User
    _on_unsubscribe: Optional[Callable[[], None]] = None

    def on_unsubscribe(self, cb: Callable[[], None]) -> None:
        self._on_unsubscribe = cb

    async def emit(self, data, to: Optional[str] = None) -> None:
        to = to or self.sid
        await self.sio.emit(self.path, data, to=to)


class OnSubscribe(Protocol):
    def __call__(self, req: WatchRequest, **path_params):
        ...


@dataclass
class Watch:
    func: OnSubscribe
    prefix: str
    path: str


@dataclass
class SioRoute:
    pattern: Pattern
    func: OnSubscribe


class FastIORouter(APIRouter):
    """
    This is a router that extends on the default fastapi router. It adds a new virtual
    method "watch". The method registers a socketio endpoint along with a GET endpoint,
    it requires a "target" observable and a function that returns a dict containing
    the path parameters or `None`. If `None` is returns, the data won't be available
    in any rooms/routes.

    The GET endpoint created is automatically registered to fastapi, including the
    the openapi docs.

    In a way this is like a "reverse" get request, normally in a get request, fastapi
    will parse the url and other params and provide it to the decorated function, the
    function is then expected to return the response. In the "watch" method, the flow
    is reversed, FastIO will provide the response, and the decorated function should
    return the path parameters (query and body parameters are not supported).

    .. code-block::
        import rx.subject
        import uvicorn

        from api_server.fast_io import FastIORouter

        class ReturnVideo(pydantic.BaseModel):
            film_title: str

        app = FastIO()
        router = FastIORouter()
        target = rx.subject.Subject()


        @router.watch("/video_rental/{film_title}/available", target)
        def watch_availability(rental: dict):
            return {"film_title": rental["film"]}, rental


        @router.post("/video_rental/return_video")
        def post_return_video(return_video: ReturnVideo):
            target.on_next({"film": return_video.film_title, "available": True})


        app.include_router(router)

        uvicorn.run(app)
    """

    def __init__(self, *args, user_dep: Optional[Callable[..., User]] = None, **kwargs):
        super().__init__(*args, **kwargs)
        self.user_dep = user_dep or (lambda: User(username="stub", is_admin=True))
        self.watches = cast(List[Watch], [])
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def include_router(self, router: "FastIORouter", **kwargs):
        super().include_router(router, **kwargs)
        prefix = kwargs.get("prefix", "")

        for watch in router.watches:
            self.watches.append(
                Watch(
                    func=watch.func,
                    prefix=prefix + watch.prefix,
                    path=prefix + watch.path,
                )
            )

    def watch(self, path: str):
        """
        Registers a socket.io and rest GET endpoint. The decorated function should
        take in the event from the target observable and returns either `None` or a
        tuple containing a dict of the path parameters that the event should be
        bind to and the response. The response must be a dict or a pydantic model.

        :param data_store: default is `MemoryDataStore`.
        """

        def decorator(func: OnSubscribe) -> DecoratedCallable:
            self.watches.append(
                Watch(func=func, prefix=self.prefix, path=self.prefix + path)
            )

        return decorator


def make_sio(authenticator: Optional[JwtAuthenticator], logger: logging.Logger):
    sio = socketio.AsyncServer(
        async_mode="asgi", cors_allowed_origins="*", logger=logger
    )

    async def on_connect(sid: str, environ: dict, auth: Optional[dict] = None):
        logger.info(
            f'[{sid}] new connection from "{environ["REMOTE_ADDR"]}:{environ["REMOTE_PORT"]}"'
        )

        async with sio.session(sid) as session:
            session["subscriptions"] = {}
            if not authenticator:
                session["user"] = User(username="stub", is_admin=True)
                return True

            try:
                if auth is None:
                    raise AuthenticationError("no auth options provided")
                if "token" not in auth:
                    raise AuthenticationError("no token provided")
                user = await authenticator.verify_token(auth["token"])
                session["user"] = user
                return True
            except AuthenticationError as e:
                logger.error(f"authentication failed: {e}")
                return False

    sio.on("connect", on_connect)

    return sio


class FastIO(socketio.ASGIApp):
    def __init__(
        self,
        *,
        authenticator: Optional[JwtAuthenticator] = None,
        logger: logging.Logger = None,
        **fast_api_args,
    ):
        """
        :param authenticator: Authenticator used to verify socket.io connections, for
            FastAPI endpoints, a dependency must be provided.
        """
        self.authenticator = authenticator
        self.logger = logger or logging.getLogger(self.__class__.__name__)
        self.sio = make_sio(authenticator, self.logger)
        self.fapi = FastAPI(**fast_api_args)
        super().__init__(self.sio, other_asgi_app=self.fapi)
        self._sio_routes: List[SioRoute] = []

        self.sio.on("subscribe", self._on_subscribe)
        self.sio.on("unsubscribe", self._on_unsubscribe)

    def include_router(self, router: FastIORouter, prefix: str = "", **kwargs):
        self.fapi.include_router(router, prefix=prefix, **kwargs)
        for watch in router.watches:
            pattern, _, _ = compile_path(prefix + watch.path)
            self._sio_routes.append(SioRoute(pattern, watch.func))

    @staticmethod
    def _parse_sub_data(data: dict) -> SubscriptionData:
        if "path" not in data:
            raise SubscribeError("missing 'path'")
        path = url_unquote(data["path"])
        return SubscriptionData(path=path)

    async def _on_subscribe(self, sid: str, data: dict):
        try:
            sub_data = self._parse_sub_data(data)
        except SubscribeError as e:
            await self.sio.emit("subscribe", {"success": False, "error": str(e)})
            return

        matches = ((r.pattern.match(sub_data.path), r) for r in self._sio_routes)
        try:
            match, sio_route = next((m for m in matches if m[0]))

            async with self.sio.session(sid) as session:
                user = session["user"]
                req = WatchRequest(sid=sid, sio=self.sio, path=sub_data.path, user=user)
                session["subscriptions"][sub_data.path] = req
                maybe_coro = sio_route.func(req, **match.groupdict())
                if inspect.isawaitable(maybe_coro):
                    await maybe_coro
                self.sio.enter_room(sid, sub_data.path)
        except StopIteration:
            await self.sio.emit(
                "subscribe", {"success": False, "error": "no events in path"}
            )
            return
        except HTTPException as e:
            await self.sio.emit(
                "subscribe", {"success": False, "error": f"{e.status_code} {e.detail}"}
            )

        await self.sio.emit("subscribe", {"success": True}, sid)

    async def _on_unsubscribe(self, sid: str, data: dict):
        try:
            sub_data = self._parse_sub_data(data)
            async with self.sio.session(sid) as session:
                req: WatchRequest = session["subscriptions"].get(sub_data.path, None)
                if req is None:
                    raise SubscribeError("not subscribed to topic")
                maybe_coro = req._on_unsubscribe()  # pylint: disable=protected-access
                if inspect.isawaitable(maybe_coro):
                    await maybe_coro
                self.sio.leave_room(sid, sub_data.path)
                await self.sio.emit("unsubscribe", {"success": True})
        except SubscribeError as e:
            await self.sio.emit("unsubscribe", {"success": False, "error": str(e)})
