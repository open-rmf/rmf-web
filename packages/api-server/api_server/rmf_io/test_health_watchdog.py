import logging
import unittest
from typing import Any, Callable, List

from rx import Observable
from rx.scheduler.historicalscheduler import HistoricalScheduler

from ..models import HealthStatus
from .gateway import RmfGateway
from .health_watchdog import HealthWatchdog
from .test_data import make_door_state, make_lift_state


class TestHealthWatchdog_DoorHealth(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.scheduler = HistoricalScheduler()
        self.rmf = RmfGateway()
        logger = logging.Logger("test")
        logger.setLevel("CRITICAL")
        self.health_watchdog = HealthWatchdog(
            self.rmf, scheduler=self.scheduler, logger=logger
        )

    async def test_heartbeat(self):
        class Param:
            def __init__(
                self,
                health_obs: Observable,
                source: Observable,
                factory: Callable[[str], Any],
            ):
                self.health_obs = health_obs
                self.source = source
                self.factory = factory

        def lift_state_factory(name: str):
            state = make_lift_state()
            state.lift_name = name
            return state

        params: List[Param] = [
            Param(self.rmf.door_health, self.rmf.door_states, make_door_state),
            Param(self.rmf.lift_health, self.rmf.lift_states, lift_state_factory),
        ]

        for p in params:
            with self.subTest(p=p):
                health = None

                def assign(v):
                    nonlocal health
                    health = v

                p.health_obs.pipe().subscribe(assign)

                p.source.on_next(p.factory("test"))
                self.scheduler.advance_by(0)
                self.assertEqual(health.health_status, HealthStatus.HEALTHY)
                self.assertEqual(health.name, "test")

                # it should not be dead yet
                self.scheduler.advance_by(HealthWatchdog.LIVELINESS / 2)
                self.assertEqual(health.health_status, HealthStatus.HEALTHY)
                self.assertEqual(health.name, "test")

                # it should be dead now because the time between states has reached the threshold
                self.scheduler.advance_by(HealthWatchdog.LIVELINESS / 2)
                self.assertEqual(health.health_status, HealthStatus.DEAD)
                self.assertEqual(health.name, "test")

                # it should become alive again when a new state is emitted
                p.source.on_next(p.factory("test"))
                self.assertEqual(health.health_status, HealthStatus.HEALTHY)
                self.assertEqual(health.name, "test")
