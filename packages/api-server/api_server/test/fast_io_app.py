import pydantic
import socketio
from rx import operators as rxops
from rx.core.typing import Observable
from rx.subject import Subject

from api_server.fast_io import FastIO, FastIORouter, WatchRequest
from api_server.routes.utils import rx_watcher

sio = socketio.AsyncServer(async_mode="asgi")
app = FastIO(sio)
router = FastIORouter()
router_with_prefix = FastIORouter(prefix="/router_with_prefix")
router_include_with_prefix = FastIORouter()
router_both_prefix = FastIORouter(prefix="/router_both_prefix")
target = Subject()
target_with_prefix = Subject()
target_include_with_prefix = Subject()
target_both_prefix = Subject()


class ReturnVideo(pydantic.BaseModel):
    film_title: str


def handler(req: WatchRequest, film_title: str, target_obs: Observable):
    rx_watcher(req, target_obs.pipe(rxops.filter(lambda x: x["film"] == film_title)))


@router.watch("/video_rental/{film_title}/available")
def router_watch_availability(req: WatchRequest, film_title: str):
    handler(req, film_title, target)


@router.post("/video_rental/return_video")
def router_post_return_video(return_video: ReturnVideo):
    target.on_next({"film": return_video.film_title, "available": True})


@router_with_prefix.watch("/video_rental/{film_title}/available")
def router_with_prefix_watch_availability(req: WatchRequest, film_title: str):
    handler(req, film_title, target_with_prefix)


@router_with_prefix.post("/video_rental/return_video")
def router_with_prefix_post_return_video(return_video: ReturnVideo):
    target_with_prefix.on_next({"film": return_video.film_title, "available": True})


@router_include_with_prefix.watch("/video_rental/{film_title}/available")
def router_include_with_prefix_watch_availability(req: WatchRequest, film_title: str):
    handler(req, film_title, target_include_with_prefix)


@router_include_with_prefix.post("/video_rental/return_video")
def router_include_with_prefix_post_return_video(return_video: ReturnVideo):
    target_include_with_prefix.on_next(
        {"film": return_video.film_title, "available": True}
    )


@router_both_prefix.watch("/video_rental/{film_title}/available")
def router_both_prefix_watch_availability(req: WatchRequest, film_title: str):
    handler(req, film_title, target_both_prefix)


@router_both_prefix.post("/video_rental/return_video")
def router_both_prefix_post_return_video(return_video: ReturnVideo):
    target_both_prefix.on_next({"film": return_video.film_title, "available": True})


app.include_router(router)
app.include_router(router_with_prefix)
app.include_router(router_include_with_prefix, prefix="/router_include_with_prefix")
app.include_router(router_both_prefix, prefix="/include_prefix")
