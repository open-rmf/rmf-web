import asyncio
import unittest

from rx.operators import first
from rx.scheduler.historicalscheduler import HistoricalScheduler

from ..models import DoorHealth, HealthStatus
from .gateway import RmfGateway
from .health_watchdog import HealthWatchdog
from .test_data import make_door_state


class TestHealthWatchdog_DoorHealth(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.scheduler = HistoricalScheduler()
        self.rmf = RmfGateway()
        self.health_watchdog = HealthWatchdog(self.rmf, scheduler=self.scheduler)

    async def test_heartbeat(self):
        # first, it will start with unknown health status, when we publish a door state,
        # the door will be alive only after LIVELINESS, because the implementation only looks at
        # the number of events emitted in a given window, so it only "ticks" at LIVELINESS.
        self.rmf.door_states.on_next(make_door_state("test_door"))
        self.scheduler.advance_by(HealthWatchdog.LIVELINESS)
        fut = asyncio.Future()
        self.rmf.door_health.pipe(first()).subscribe(fut.set_result)
        health: DoorHealth = await fut
        self.assertEqual(health.health_status, HealthStatus.HEALTHY)
        self.assertEqual(health.name, "test_door")

        # now, it should be dead because there is no new events emitted
        self.scheduler.advance_by(HealthWatchdog.LIVELINESS * 2)
        fut = asyncio.Future()
        self.rmf.door_health.pipe(first()).subscribe(fut.set_result)
        health: DoorHealth = await fut
        self.assertEqual(health.health_status, HealthStatus.DEAD)
        self.assertEqual(health.name, "test_door")
