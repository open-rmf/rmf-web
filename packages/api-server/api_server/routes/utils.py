import asyncio

from rx import operators as rxops
from rx.core.typing import Observable
from rx.scheduler.eventloop.asyncioscheduler import AsyncIOScheduler

from api_server.fast_io import WatchRequest


def rx_watcher(req: WatchRequest, target: Observable):
    async def on_next(data):
        await req.emit(data)

    loop = asyncio.get_event_loop()
    sub = target.pipe(rxops.observe_on(AsyncIOScheduler(loop))).subscribe(
        lambda data: loop.create_task(on_next(data)),
    )
    req.on_unsubscribe(sub.dispose)
