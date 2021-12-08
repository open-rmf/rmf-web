from typing import Dict, cast

import pydantic
import socketio
from rx import operators as rxops
from rx.subject.subject import Subject

from api_server.fast_io import FastIO, FastIORouter, SubscriptionRequest

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


@router.sub("/video_rental/{film_title}/available", response_model=ReturnVideo)
def router_sub_availability(_req: SubscriptionRequest, film_title: str):
    return target.pipe(
        rxops.filter(lambda x: cast(ReturnVideo, x).film_title == film_title)
    )


@router.post("/video_rental/return_video")
def router_post_return_video(return_video: ReturnVideo):
    target.on_next(return_video)


@router_with_prefix.sub(
    "/video_rental/{film_title}/available", response_model=ReturnVideo
)
def router_with_prefix_sub_availability(_req: SubscriptionRequest, film_title: str):
    return target_with_prefix.pipe(
        rxops.filter(lambda x: cast(ReturnVideo, x).film_title == film_title)
    )


@router_with_prefix.post("/video_rental/return_video")
def router_with_prefix_post_return_video(return_video: ReturnVideo):
    target_with_prefix.on_next(return_video)


@router_include_with_prefix.sub(
    "/video_rental/{film_title}/available", response_model=ReturnVideo
)
def router_include_with_prefix_sub_availability(
    _req: SubscriptionRequest, film_title: str
):
    return target_include_with_prefix.pipe(
        rxops.filter(lambda x: cast(ReturnVideo, x).film_title == film_title)
    )


@router_include_with_prefix.post("/video_rental/return_video")
def router_include_with_prefix_post_return_video(return_video: ReturnVideo):
    target_include_with_prefix.on_next(return_video)


@router_both_prefix.sub(
    "/video_rental/{film_title}/available", response_model=ReturnVideo
)
def router_both_prefix_sub_availability(_req: SubscriptionRequest, film_title: str):
    return target_both_prefix.pipe(
        rxops.filter(lambda x: cast(ReturnVideo, x).film_title == film_title)
    )


@router_both_prefix.post("/video_rental/return_video")
def router_both_prefix_post_return_video(return_video: ReturnVideo):
    target_both_prefix.on_next(return_video)


app.include_router(router)
app.include_router(router_with_prefix)
app.include_router(router_include_with_prefix, prefix="/router_include_with_prefix")
app.include_router(router_both_prefix, prefix="/include_prefix")
