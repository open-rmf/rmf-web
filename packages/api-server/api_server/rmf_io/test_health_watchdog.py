import logging
import unittest
from typing import Any, Callable, Optional

from rmf_door_msgs.msg import DoorMode
from rx import Observable
from rx.scheduler.historicalscheduler import HistoricalScheduler

from ..models import (
    DispenserHealth,
    DoorHealth,
    HealthStatus,
    LiftHealth,
    RobotHealth,
    get_robot_id,
)
from .gateway import RmfGateway
from .health_watchdog import HealthWatchdog
from .test_data import (
    make_building_map,
    make_dispenser_state,
    make_door,
    make_door_state,
    make_fleet_state,
    make_lift,
    make_lift_state,
    make_robot_state,
)


class BaseHealthWatchdogTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.scheduler = HistoricalScheduler()
        self.rmf = RmfGateway()

        self.logger = logging.Logger("test")
        self.logger.setLevel("CRITICAL")
        self.health_watchdog = HealthWatchdog(
            self.rmf, scheduler=self.scheduler, logger=self.logger
        )
        self.scheduler.advance_by(1)


async def test_heartbeat(
    test: BaseHealthWatchdogTests,
    health_obs: Observable,
    source: Observable,
    factory: Callable[[str], Any],
):
    """
    :param health_obs: Observable[BasicHealthModel], the observable of the heartbeat events
    :param source: Observable[SourceType], the observable that should trigger heartbeat events
    :param factory: Callable[[str], SourceType], a factory function that returns an object of
        the source observable sequence type
    """
    health = None

    def assign(v):
        nonlocal health
        health = v

    health_obs.subscribe(assign)

    source.on_next(factory("test_id"))
    test.scheduler.advance_by(0)
    test.assertEqual(health.health_status, HealthStatus.HEALTHY)
    test.assertEqual(health.id_, "test_id")

    # it should not be dead yet
    test.scheduler.advance_by(HealthWatchdog.LIVELINESS / 2)
    test.assertEqual(health.health_status, HealthStatus.HEALTHY)
    test.assertEqual(health.id_, "test_id")

    # # it should be dead now because the time between states has reached the threshold
    test.scheduler.advance_by(HealthWatchdog.LIVELINESS / 2)
    test.assertEqual(health.health_status, HealthStatus.DEAD)
    test.assertEqual(health.id_, "test_id")

    # # it should become alive again when a new state is emitted
    source.on_next(factory("test_id"))
    test.assertEqual(health.health_status, HealthStatus.HEALTHY)
    test.assertEqual(health.id_, "test_id")


class TestHealthWatchdog_DoorHealth(BaseHealthWatchdogTests):
    async def test_heartbeat(self):
        await test_heartbeat(
            self, self.rmf.door_health, self.rmf.door_states, make_door_state
        )

    async def test_door_mode_offline_is_unhealthy(self):
        health: DoorHealth = None

        def on_next(v):
            nonlocal health
            health = v

        self.rmf.door_health.subscribe(on_next)

        state = make_door_state("test_door", DoorMode.MODE_OFFLINE)
        self.rmf.door_states.on_next(state)
        self.scheduler.advance_by(0)
        self.assertEqual(health.health_status, HealthStatus.UNHEALTHY)

    async def test_door_mode_unknown_is_unhealthy(self):
        health: DoorHealth = None

        def on_next(v):
            nonlocal health
            health = v

        self.rmf.door_health.subscribe(on_next)

        state = make_door_state("test_door", DoorMode.MODE_UNKNOWN)
        self.rmf.door_states.on_next(state)
        self.scheduler.advance_by(0)
        self.assertEqual(health.health_status, HealthStatus.UNHEALTHY)

    async def test_heartbeat_with_no_state(self):
        building_map = make_building_map()
        building_map.levels[0].doors = [make_door("test_door")]
        self.rmf.building_map.on_next(building_map)

        health: Optional[DoorHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.door_health.subscribe(assign)

        self.scheduler.advance_by(self.health_watchdog.LIVELINESS)
        self.assertEqual(health.id_, "test_door")
        self.assertEqual(health.health_status, HealthStatus.DEAD)


class TestHealthWatchdog_LiftHealth(BaseHealthWatchdogTests):
    async def test_heartbeat(self):
        await test_heartbeat(
            self, self.rmf.lift_health, self.rmf.lift_states, make_lift_state
        )

    async def test_heartbeat_with_no_state(self):
        """
        Tests that a lift that never sends any state can be caught by the heartbeat watchdog.
        This also tests that the list of known lifts is automatically updated when a new
        building map comes in.
        """
        building_map = make_building_map()
        building_map.lifts = [make_lift("test_lift")]
        self.rmf.building_map.on_next(building_map)

        health: Optional[LiftHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.lift_health.subscribe(assign)

        self.scheduler.advance_by(self.health_watchdog.LIVELINESS)
        self.assertEqual(health.id_, "test_lift")
        self.assertEqual(health.health_status, HealthStatus.DEAD)


class TestHealthWatchdog_DispenserHealth(BaseHealthWatchdogTests):
    async def test_heartbeat(self):
        await test_heartbeat(
            self,
            self.rmf.dispenser_health,
            self.rmf.dispenser_states,
            make_dispenser_state,
        )

    async def test_heartbeat_with_no_state(self):
        # simulate the situation where a dispenser state loaded from persistent
        # store never send any states.
        self.rmf.dispenser_states.on_next(make_dispenser_state("test_dispenser"))
        self.health_watchdog = HealthWatchdog(
            self.rmf, scheduler=self.scheduler, logger=self.logger
        )

        health: Optional[DispenserHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.dispenser_health.subscribe(assign)

        self.scheduler.advance_by(self.health_watchdog.LIVELINESS)
        self.assertEqual(health.id_, "test_dispenser")
        self.assertEqual(health.health_status, HealthStatus.DEAD)


class TestHealthWatchdog_RobotHealth(BaseHealthWatchdogTests):
    async def test_heartbeat(self):
        health = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.robot_health.subscribe(assign)

        def factory():
            state = make_fleet_state("test_fleet")
            state.robots = [make_robot_state("test_robot")]
            return state

        robot_id = get_robot_id("test_fleet", "test_robot")
        self.rmf.fleet_states.on_next(factory())
        self.scheduler.advance_by(0)
        self.assertEqual(health.health_status, HealthStatus.HEALTHY)
        self.assertEqual(health.id_, robot_id)

        # it should not be dead yet
        self.scheduler.advance_by(HealthWatchdog.LIVELINESS / 2)
        self.assertEqual(health.health_status, HealthStatus.HEALTHY)
        self.assertEqual(health.id_, robot_id)

        # # it should be dead now because the time between states has reached the threshold
        self.scheduler.advance_by(HealthWatchdog.LIVELINESS / 2)
        self.assertEqual(health.health_status, HealthStatus.DEAD)
        self.assertEqual(health.id_, robot_id)

        # # it should become alive again when a new state is emitted
        self.rmf.fleet_states.on_next(factory())
        self.assertEqual(health.health_status, HealthStatus.HEALTHY)
        self.assertEqual(health.id_, robot_id)

    async def test_heartbeat_with_no_state(self):
        # simulate the situation where a fleet state loaded from persistent
        # store never send any states.
        def factory():
            state = make_fleet_state("test_fleet")
            state.robots = [make_robot_state("test_robot")]
            return state

        self.rmf.fleet_states.on_next(factory())
        self.health_watchdog = HealthWatchdog(
            self.rmf, scheduler=self.scheduler, logger=self.logger
        )

        health: Optional[RobotHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.robot_health.subscribe(assign)

        robot_id = get_robot_id("test_fleet", "test_robot")
        self.scheduler.advance_by(self.health_watchdog.LIVELINESS)
        self.assertEqual(health.id_, robot_id)
        self.assertEqual(health.health_status, HealthStatus.DEAD)
