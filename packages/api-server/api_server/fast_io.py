import asyncio
import inspect
import logging
from dataclasses import dataclass
from re import Pattern
from typing import Any, Callable, Dict, List, Optional, cast
from urllib.parse import unquote as url_unquote

import pydantic
import socketio
from fastapi import APIRouter, FastAPI, Request
from fastapi.exceptions import HTTPException
from fastapi.types import DecoratedCallable
from rx import Observable
from starlette.routing import compile_path, replace_params

from .authenticator import AuthenticationError, JwtAuthenticator


@dataclass
class Watch:
    target: Observable
    prefix: str
    path: str
    sticky: bool
    on_subscribe: Callable[[str], Any]
    on_data: Callable[[Any], Any]


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

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.watches = cast(List[Watch], [])
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def include_router(self, router: "FastIORouter", **kwargs):
        super().include_router(router, **kwargs)
        prefix = kwargs.get("prefix", "")

        for watch in router.watches:
            self.watches.append(
                Watch(
                    target=watch.target,
                    prefix=prefix + watch.prefix,
                    path=prefix + watch.path,
                    sticky=watch.sticky,
                    on_subscribe=watch.on_subscribe,
                    on_data=watch.on_data,
                )
            )

    def watch(self, path: str, target: Observable, *args, sticky=True, **kwargs):
        """
        Registers a socket.io and rest GET endpoint. The decorated function should
        take in the event from the target observable and returns either `None` or a
        tuple containing a dict of the path parameters that the event should be
        bind to and the response. The response must be a dict or a pydantic model.

        :param sticky: If true, the socket.io endpoint will automatically send the
        latest event to new clients and the generated GET endpoint will return the
        latest event. If false, a GET endpoint will still be generated but it will
        always return 422 error, this is useful for documentation purposes.
        """
        get_deco = super().get(path, *args, **kwargs)

        def decorator(func: DecoratedCallable) -> DecoratedCallable:
            prefixed_path = self.prefix + path
            _, _, convertors = compile_path(prefixed_path)
            room_data = {}

            def on_subscribe(path: str):
                return room_data.get(path, None)

            async def on_data(data):
                if asyncio.iscoroutinefunction(func):
                    result = await func(data)
                else:
                    result = func(data)
                if result is None:
                    return None
                path_params, resp = result
                room_id, _ = replace_params(path, convertors, path_params)
                if isinstance(resp, pydantic.BaseModel):
                    resp = resp.dict()
                room_data[room_id] = resp
                return room_id, resp

            watch = Watch(
                target=target,
                prefix=self.prefix,
                path=self.prefix + path,
                sticky=sticky,
                on_subscribe=on_subscribe,
                on_data=on_data,
            )
            self.watches.append(watch)

            def get_func(**path_params):
                if not sticky:
                    raise HTTPException(422, "only available in socket.io")
                room_id, _ = replace_params(path, convertors, path_params)
                try:
                    return room_data[room_id]
                except KeyError as e:
                    raise HTTPException(404) from e

            get_func.__name__ = func.__name__
            if sticky:
                get_func.__doc__ = "Available in socket.io.\n\n" + (func.__doc__ or "")
            else:
                get_func.__doc__ = "ONLY available in socket.io.\n\n" + (
                    func.__doc__ or ""
                )
            params = [
                inspect.Parameter(
                    "req", inspect.Parameter.KEYWORD_ONLY, annotation=Request
                )
            ]
            params.extend(
                [
                    inspect.Parameter(k, inspect.Parameter.KEYWORD_ONLY, annotation=str)
                    for k in convertors
                ]
            )
            get_func.__signature__ = inspect.Signature(params)

            get_deco(get_func)

        return decorator


def make_sio(authenticator: Optional[JwtAuthenticator], logger: logging.Logger):
    def on_connect(sid: str, environ: dict, auth: Optional[dict] = None):
        logger.info(
            f'[{sid}] new connection from "{environ["REMOTE_ADDR"]}:{environ["REMOTE_PORT"]}"'
        )

        if not authenticator:
            return True

        try:
            if auth is None:
                raise AuthenticationError("no auth options provided")
            if "token" not in auth:
                raise AuthenticationError("no token provided")
            authenticator.verify_token(auth["token"])
            return True
        except AuthenticationError as e:
            logger.error(f"authentication failed: {e}")
            return False

    sio = socketio.AsyncServer(
        async_mode="asgi", cors_allowed_origins="*", logger=logger
    )
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
        self.logger = logger or logging.getLogger(self.__class__.__name__)
        self.sio = make_sio(authenticator, self.logger)
        self.fapi = FastAPI(**fast_api_args)
        super().__init__(self.sio, other_asgi_app=self.fapi)
        self._router = FastIORouter()
        self._patterns = cast(Dict[Pattern, Watch], {})

        self.sio.on("subscribe", self._on_subscribe)
        self.fapi.add_event_handler("startup", self._on_startup)

    def include_router(self, router: FastIORouter, prefix: str = "", **kwargs):
        self._router.include_router(router, prefix=prefix, **kwargs)
        self.fapi.include_router(router, prefix=prefix, **kwargs)

    def _on_startup(self):
        loop = asyncio.get_event_loop()

        for watch in self._router.watches:
            pattern, _, _ = compile_path(watch.path)
            self._patterns[pattern] = watch

            async def on_next(data, watch=watch):
                result = await watch.on_data(data)
                if result is None:
                    return
                room_id, resp = result
                await self.sio.emit(watch.prefix + room_id, resp)

            watch.target.subscribe(
                lambda data, on_next=on_next: loop.create_task(on_next(data))
            )

    async def _on_subscribe(self, sid: str, data: dict):
        if "path" not in data:
            await self.sio.emit(
                "subscribe", {"success": False, "error": "missing 'path'"}
            )
            return

        path = url_unquote(data["path"])
        try:
            watch = next(
                watch
                for pattern, watch in self._patterns.items()
                if pattern.match(path)
            )
        except StopIteration:
            await self.sio.emit(
                "subscribe", {"success": False, "error": "no events in path"}
            )
            return

        await self.sio.emit("subscribe", {"success": True}, sid)
        if not watch.sticky:
            return
        stripped_path = path[len(watch.prefix) :]
        current = watch.on_subscribe(stripped_path)
        if current is None:
            return
        if isinstance(current, pydantic.BaseModel):
            current = current.dict()
        await self.sio.emit(path, current, to=sid)
