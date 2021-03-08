import asyncio
import unittest
from typing import Any, Callable
from unittest.mock import MagicMock, call

from rx import Observable
from rx.scheduler.historicalscheduler import HistoricalScheduler

from ..models import DoorHealth, HealthStatus, LiftHealth
from ..repositories import RmfRepository
from .book_keeper import RmfBookKeeper
from .gateway import RmfGateway
from .test_data import make_building_map, make_door, make_door_state, make_lift_state


class RmfBookKeeperFixture(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.rmf = RmfGateway()
        self.repo = MagicMock(RmfRepository)
        self.scheduler = HistoricalScheduler()
        self.book_keeper = RmfBookKeeper(self.rmf, self.repo, scheduler=self.scheduler)
        self.book_keeper.start()


def make_base_test(
    get_source: Callable[[RmfGateway], Observable],
    factory: Callable[[str], Any],
    frequency: float,
    get_update_mock: Callable[[RmfRepository], Any],
):
    """
    Args:
        factory: Return value must be the requirement:
            `factory("foo") is factory("foo") == True`
        frequency: Must be > 0
    """

    class BaseTest(RmfBookKeeperFixture):
        def setUp(self):
            super().setUp()
            self.source = get_source(self.rmf)
            self.update_mock = get_update_mock(self.repo)

        async def test_write_frequency(self):
            self.source.on_next(factory("test_id"))
            self.scheduler.advance_by(0.9 * frequency)
            # shouldn't write before t = 1
            self.update_mock.assert_not_called()
            self.scheduler.advance_by(0.1 * frequency)
            await asyncio.sleep(0)
            self.update_mock.assert_awaited_once()

            self.source.on_next(factory("test_id"))
            self.scheduler.advance_by(0.9 * frequency)
            # shouldn't write again before t = 2
            self.update_mock.assert_called_once()
            self.scheduler.advance_by(0.1 * frequency)
            await asyncio.sleep(0)
            self.assertEqual(self.update_mock.await_count, 2)

        async def test_write_latest(self):
            """
            If new values are received before due time, should update only the latest value
            """
            value = factory("test_id")
            self.source.on_next(value)
            self.scheduler.advance_by(0.9 * frequency)
            value_2 = factory("test_id")
            self.source.on_next(value_2)
            self.scheduler.advance_by(0.1 * frequency)

            await asyncio.sleep(0)

            class Matcher:
                def __eq__(self, other):
                    return other is value_2

            self.update_mock.assert_awaited_once_with(Matcher())

        async def test_write_latest_multiple(self):
            """
            If new values for different keys are received before due time,
            the latest value of each key should be written.
            """
            value = factory("test_id")
            self.source.on_next(value)
            value2 = factory("test_id_2")
            self.source.on_next(value2)

            self.scheduler.advance_by(0.9 * frequency)
            value_2 = factory("test_id")
            self.source.on_next(value_2)
            value2_2 = factory("test_id_2")
            self.source.on_next(value2_2)

            self.scheduler.advance_by(0.1 * frequency)

            class Matcher:
                def __init__(self, val):
                    self.val = val

                def __eq__(self, other):
                    return other is self.val

            await asyncio.sleep(0)
            calls = [
                call(Matcher(value_2)),
                call(Matcher(value2_2)),
            ]
            self.assertEqual(self.update_mock.await_count, 2)
            self.update_mock.assert_has_awaits(calls)

    return BaseTest


class TestRmfBookKeeper_DoorStates(
    make_base_test(
        lambda x: x.door_states,
        make_door_state,
        1,
        lambda x: x.update_door_state,
    )
):
    pass


class TestRmfBookKeeper_LiftStates(
    make_base_test(
        lambda x: x.lift_states,
        make_lift_state,
        1,
        lambda x: x.update_lift_state,
    )
):
    pass


class TestRmfBookKeeper_DoorHealth(RmfBookKeeperFixture):
    async def test_write_door_health(self):
        """
        All health changes should be written, regardless of time between them.
        """
        door_health = DoorHealth(name="test_door", health_status=HealthStatus.DEAD)
        self.rmf.door_health.on_next(door_health)
        self.scheduler.advance_by(0)
        await asyncio.sleep(0)
        self.repo.update_door_health.assert_awaited()

        door_health = DoorHealth(name="test_door", health_status=HealthStatus.HEALTHY)
        self.rmf.door_health.on_next(door_health)
        self.scheduler.advance_by(0)
        await asyncio.sleep(0)
        self.assertEqual(2, self.repo.update_door_health.await_count)


class TestRmfBookKeeper_LiftHealth(RmfBookKeeperFixture):
    async def test_write_lift_health(self):
        """
        All health changes should be written, regardless of time between them.
        """
        lift_health = LiftHealth(name="test_lift", health_status=HealthStatus.DEAD)
        self.rmf.lift_health.on_next(lift_health)
        self.scheduler.advance_by(0)
        await asyncio.sleep(0)
        self.repo.update_lift_health.assert_awaited()

        lift_health = LiftHealth(name="test_lift", health_status=HealthStatus.HEALTHY)
        self.rmf.lift_health.on_next(lift_health)
        self.scheduler.advance_by(0)
        await asyncio.sleep(0)
        self.assertEqual(2, self.repo.update_lift_health.await_count)


class TestRmfBookKeeper_BuildingMap(RmfBookKeeperFixture):
    async def test_write_doors(self):
        """
        doors should be written when new building map is received
        """
        building_map = make_building_map()
        building_map.levels[0].doors.append(make_door("test_door"))
        building_map.levels[0].doors.append(make_door("test_door2"))
        self.rmf.building_map.on_next(building_map)
        self.scheduler.advance_by(0)
        await asyncio.sleep(0)
        self.repo.sync_doors.assert_awaited()
