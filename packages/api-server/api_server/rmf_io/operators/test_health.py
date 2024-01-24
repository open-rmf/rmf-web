import unittest
from datetime import datetime
from typing import Any, Optional, cast

import reactivex as rx
from reactivex import operators as ops
from reactivex.scheduler.historicalscheduler import HistoricalScheduler

from ...models import BasicHealth, HealthStatus
from .health import most_critical


class TestHealth(BasicHealth):
    @staticmethod
    async def from_tortoise_orm(_tortoise):
        raise NotImplementedError()


class TestMostCritical(unittest.TestCase):
    def test_returns_dead_over_unhealthy(self):
        healths = [
            TestHealth(id_="test", health_status=HealthStatus.DEAD, health_message=""),
            TestHealth(
                id_="test", health_status=HealthStatus.UNHEALTHY, health_message=""
            ),
        ]
        obs_a = rx.of(healths[0]).pipe(
            ops.timestamp(scheduler=HistoricalScheduler(datetime.fromtimestamp(1)))
        )
        obs_b = rx.of(healths[1]).pipe(
            ops.timestamp(scheduler=HistoricalScheduler(datetime.fromtimestamp(2)))
        )

        result: TestHealth | None = None

        def assign(v):
            nonlocal result
            result = v

        obs_a.pipe(cast(Any, ops.combine_latest(obs_b)), most_critical()).subscribe(
            assign
        )
        self.assertIsNotNone(result)
        result = cast(TestHealth, result)
        self.assertEqual(result.health_status, HealthStatus.DEAD)

    def test_return_most_recent(self):
        healths = [
            TestHealth(
                id_="test",
                health_status=HealthStatus.DEAD,
                health_message="first",
            ),
            TestHealth(
                id_="test",
                health_status=HealthStatus.DEAD,
                health_message="second",
            ),
        ]
        obs_a = rx.of(healths[0]).pipe(
            ops.timestamp(scheduler=HistoricalScheduler(datetime.fromtimestamp(1)))
        )
        obs_b = rx.of(healths[1]).pipe(
            ops.timestamp(scheduler=HistoricalScheduler(datetime.fromtimestamp(2)))
        )

        result: Optional[TestHealth] = None

        def assign(v):
            nonlocal result
            result = v

        obs_a.pipe(cast(Any, ops.combine_latest(obs_b)), most_critical()).subscribe(
            assign
        )
        self.assertIsNotNone(result)
        result = cast(TestHealth, result)
        self.assertEqual(result.health_status, HealthStatus.DEAD)
        self.assertEqual(result.health_message, "second")

    def test_ignore_none_values(self):
        healths = [
            TestHealth(
                id_="test", health_status=HealthStatus.HEALTHY, health_message=""
            ),
            None,
        ]
        obs_a = rx.of(healths[0]).pipe(
            ops.timestamp(scheduler=HistoricalScheduler(datetime.fromtimestamp(1)))
        )
        obs_b = rx.of(healths[1]).pipe(
            ops.timestamp(scheduler=HistoricalScheduler(datetime.fromtimestamp(2)))
        )

        result: Optional[TestHealth] = None

        def assign(v):
            nonlocal result
            result = v

        obs_a.pipe(cast(Any, ops.combine_latest(obs_b)), most_critical()).subscribe(
            assign
        )
        self.assertIsNotNone(result)
        result = cast(TestHealth, result)
        self.assertEqual(result.health_status, HealthStatus.HEALTHY)
