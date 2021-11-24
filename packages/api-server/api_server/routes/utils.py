import asyncio

from rx import operators as rxops
from rx.core.observable.observable import Observable
from rx.scheduler.eventloop.asyncioscheduler import AsyncIOScheduler

from api_server.fast_io import WatchRequest


def rx_watcher(req: WatchRequest, target: Observable):
    def handle(data):
        async def emit(data):
            await req.emit(data)

        loop.create_task(emit(data))

    loop = asyncio.get_event_loop()
    sub = target.pipe(rxops.observe_on(AsyncIOScheduler(loop))).subscribe(handle)
    req.on_unsubscribe(sub.dispose)
