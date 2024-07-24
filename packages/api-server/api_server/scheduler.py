import asyncio
from contextlib import asynccontextmanager

import schedule

from api_server.fast_io import singleton_dep


@singleton_dep
@asynccontextmanager
async def get_scheduler():
    scheduler = schedule.Scheduler()

    async def _spin_scheduler():
        while True:
            scheduler.run_pending()
            await asyncio.sleep(1)

    spin_task = asyncio.create_task(_spin_scheduler())

    yield scheduler

    try:
        spin_task.cancel()
        await spin_task
    except asyncio.CancelledError:
        pass
