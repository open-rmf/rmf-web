import asyncio
import inspect
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from re import Pattern
from typing import (
    AsyncContextManager,
    Callable,
    Dict,
    Generic,
    List,
    Optional,
    Protocol,
    Sequence,
    TypedDict,
    TypeVar,
    Union,
    cast,
)
from urllib.parse import unquote as url_unquote

import socketio
from fastapi import APIRouter, Depends, FastAPI
from fastapi.exceptions import HTTPException
from fastapi.types import DecoratedCallable
from socketio.asyncio_server import AsyncServer
from starlette.routing import compile_path

from api_server.authenticator import (
    AuthenticationError,
    JwtAuthenticator,
    StubAuthenticator,
)
from api_server.models import User

from .errors import *

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
    _subscribe_task: Optional[asyncio.Task] = None

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


class Session(TypedDict, total=False):
    user: User
    subscriptions: Dict[str, WatchRequest]


# For typing purposes
class _CustomSio(ABC, socketio.AsyncServer):
    @abstractmethod
    def session(self, sid, namespace=None) -> AsyncContextManager[Session]:
        pass


class FastIORouter(APIRouter):
    """
    This is a router that extends on the default fastapi router. It adds a new virtual
    method "watch" which registers a socket.io endpoint.

    .. code-block::
        import rx.subject
        import uvicorn

        from api_server.fast_io import FastIORouter

        app = FastIO()
        router = FastIORouter()
        watches = {}


        @router.watch("/film_rental/{film}/available")
        def router_watch_availability(req: WatchRequest, film: str):
            watches[film] = req


        @router.post("/film_rental/{film}/return")
        def router_post_return_video(film: str):
            watch_req = watches.get(film, None)
            if watch_req:
                watch_req.emit({ "available": True }, to=film)


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
        Registers a socket.io endpoint.
        """

        def decorator(func: OnSubscribe) -> DecoratedCallable:
            self.watches.append(
                Watch(func=func, prefix=self.prefix, path=self.prefix + path)
            )

        return decorator


class FastIO(socketio.ASGIApp):
    def __init__(
        self,
        sio: AsyncServer,
        *,
        authenticator: Optional[JwtAuthenticator] = None,
        logger: logging.Logger = None,
        dependencies: Optional[Sequence[Depends]] = None,
        **fast_api_args,
    ):
        """
        :param authenticator: Authenticator used to verify socket.io connections, for
            FastAPI endpoints, a dependency must be provided.
        """
        self.logger = logger or logging.getLogger(self.__class__.__name__)
        self.sio = cast(_CustomSio, sio)
        self.authenticator = authenticator or StubAuthenticator()
        self.auth_dep = self.authenticator.fastapi_dep()
        dependencies = dependencies or []
        dependencies.append(Depends(self.auth_dep))
        self.fapi = FastAPI(dependencies=dependencies, **fast_api_args)
        super().__init__(self.sio, other_asgi_app=self.fapi)
        self._sio_routes: List[SioRoute] = []

        async def on_connect(sid: str, _environ: dict, auth: Optional[dict] = None):
            async with self.sio.session(sid) as session:
                session["subscriptions"] = {}
                if not self.authenticator:
                    session["user"] = User(username="stub", is_admin=True)
                    return True
                token = None
                if auth:
                    token = auth["token"]

                try:
                    user = await self.authenticator.verify_token(token)
                    session["user"] = user
                    return True
                except AuthenticationError as e:
                    logger.error(f"authentication failed: {e}")
                    return False

        sio.on("connect", on_connect)

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

            # pylint: disable=protected-access
            async with self.sio.session(sid) as session:
                user = session["user"]
                req = WatchRequest(sid=sid, sio=self.sio, path=sub_data.path, user=user)
                session["subscriptions"][sub_data.path] = req
                maybe_coro = sio_route.func(req, **match.groupdict())
                if asyncio.iscoroutine(maybe_coro):
                    req._subscribe_task = asyncio.create_task(maybe_coro)
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
            # pylint: disable=protected-access
            async with self.sio.session(sid) as session:
                req: WatchRequest = session["subscriptions"].get(sub_data.path, None)
                if req is None:
                    raise SubscribeError("not subscribed to topic")
                if req._subscribe_task and not req._subscribe_task.done():
                    await asyncio.wait([req._subscribe_task])
                if req._on_unsubscribe:
                    maybe_coro = req._on_unsubscribe()
                    if inspect.isawaitable(maybe_coro):
                        await maybe_coro
                self.sio.leave_room(sid, sub_data.path)
                await self.sio.emit("unsubscribe", {"success": True})
        except SubscribeError as e:
            await self.sio.emit("unsubscribe", {"success": False, "error": str(e)})
