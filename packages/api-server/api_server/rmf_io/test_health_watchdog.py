import logging
import unittest
from typing import Any, Callable, Optional

from rmf_dispenser_msgs.msg import DispenserState
from rmf_door_msgs.msg import DoorMode
from rmf_fleet_msgs.msg import RobotMode
from rmf_ingestor_msgs.msg import IngestorState
from rmf_lift_msgs.msg import LiftState
from rx import Observable
from rx.scheduler.historicalscheduler import HistoricalScheduler

from ..models import tortoise_models as ttm
from . import test_data
from .gateway import RmfGateway
from .health_watchdog import HealthWatchdog


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
    test.assertEqual(health.health_status, ttm.HealthStatus.HEALTHY)
    test.assertEqual(health.id_, "test_id")

    # it should not be dead yet
    test.scheduler.advance_by(HealthWatchdog.LIVELINESS / 2)
    test.assertEqual(health.health_status, ttm.HealthStatus.HEALTHY)
    test.assertEqual(health.id_, "test_id")

    # # it should be dead now because the time between states has reached the threshold
    test.scheduler.advance_by(HealthWatchdog.LIVELINESS / 2)
    test.assertEqual(health.health_status, ttm.HealthStatus.DEAD)
    test.assertEqual(health.id_, "test_id")

    # # it should become alive again when a new state is emitted
    source.on_next(factory("test_id"))
    test.assertEqual(health.health_status, ttm.HealthStatus.HEALTHY)
    test.assertEqual(health.id_, "test_id")


class TestHealthWatchdog_DoorHealth(BaseHealthWatchdogTests):
    async def test_heartbeat(self):
        await test_heartbeat(
            self, self.rmf.door_health, self.rmf.door_states, test_data.make_door_state
        )

    async def _check_door_mode(self, mode: int, expected_health: ttm.HealthStatus):
        health: ttm.DoorHealth = None

        def on_next(v):
            nonlocal health
            health = v

        self.rmf.door_health.subscribe(on_next)

        state = test_data.make_door_state("test_door", mode)
        self.rmf.door_states.on_next(state)
        self.scheduler.advance_by(0)
        self.assertEqual(health.health_status, expected_health)

    async def test_door_mode(self):
        test_cases = [
            (DoorMode.MODE_CLOSED, ttm.HealthStatus.HEALTHY),
            (DoorMode.MODE_MOVING, ttm.HealthStatus.HEALTHY),
            (DoorMode.MODE_OPEN, ttm.HealthStatus.HEALTHY),
            (DoorMode.MODE_OFFLINE, ttm.HealthStatus.UNHEALTHY),
            (DoorMode.MODE_UNKNOWN, ttm.HealthStatus.UNHEALTHY),
            (128, ttm.HealthStatus.UNHEALTHY),
        ]
        for test in test_cases:
            await self._check_door_mode(*test)

    async def test_heartbeat_with_no_state(self):
        building_map = test_data.make_building_map()
        building_map.levels[0].doors = [test_data.make_door("test_door")]
        self.rmf.building_map.on_next(building_map)

        health: Optional[ttm.DoorHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.door_health.subscribe(assign)

        self.scheduler.advance_by(self.health_watchdog.LIVELINESS)
        self.assertEqual(health.id_, "test_door")
        self.assertEqual(health.health_status, ttm.HealthStatus.DEAD)


class TestHealthWatchdog_LiftHealth(BaseHealthWatchdogTests):
    async def test_heartbeat(self):
        await test_heartbeat(
            self, self.rmf.lift_health, self.rmf.lift_states, test_data.make_lift_state
        )

    async def test_heartbeat_with_no_state(self):
        """
        Tests that a lift that never sends any state can be caught by the heartbeat watchdog.
        This also tests that the list of known lifts is automatically updated when a new
        building map comes in.
        """
        building_map = test_data.make_building_map()
        building_map.lifts = [test_data.make_lift("test_lift")]
        self.rmf.building_map.on_next(building_map)

        health: Optional[ttm.LiftHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.lift_health.subscribe(assign)

        self.scheduler.advance_by(self.health_watchdog.LIVELINESS)
        self.assertEqual(health.id_, "test_lift")
        self.assertEqual(health.health_status, ttm.HealthStatus.DEAD)

    async def _check_lift_mode(self, mode: int, expected_health: ttm.HealthStatus):
        health: ttm.LiftHealth = None

        def on_next(v):
            nonlocal health
            health = v

        self.rmf.lift_health.subscribe(on_next)

        state = test_data.make_lift_state("test_door")
        state.current_mode = mode
        self.rmf.lift_states.on_next(state)
        self.scheduler.advance_by(0)
        self.assertEqual(health.health_status, expected_health)

    async def test_lift_mode(self):
        test_cases = [
            (LiftState.MODE_HUMAN, ttm.HealthStatus.HEALTHY),
            (LiftState.MODE_AGV, ttm.HealthStatus.HEALTHY),
            (LiftState.MODE_UNKNOWN, ttm.HealthStatus.UNHEALTHY),
            (LiftState.MODE_FIRE, ttm.HealthStatus.UNHEALTHY),
            (LiftState.MODE_EMERGENCY, ttm.HealthStatus.UNHEALTHY),
            (LiftState.MODE_OFFLINE, ttm.HealthStatus.UNHEALTHY),
            (128, ttm.HealthStatus.UNHEALTHY),
        ]
        for test in test_cases:
            await self._check_lift_mode(*test)


class TestHealthWatchdog_DispenserHealth(BaseHealthWatchdogTests):
    async def test_heartbeat(self):
        await test_heartbeat(
            self,
            self.rmf.dispenser_health,
            self.rmf.dispenser_states,
            test_data.make_dispenser_state,
        )

    async def test_heartbeat_with_no_state(self):
        # simulate the situation where a dispenser state loaded from persistent
        # store never send any states.
        self.rmf.dispenser_states.on_next(
            test_data.make_dispenser_state("test_dispenser")
        )
        self.health_watchdog = HealthWatchdog(
            self.rmf, scheduler=self.scheduler, logger=self.logger
        )

        health: Optional[ttm.DispenserHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.dispenser_health.subscribe(assign)

        self.scheduler.advance_by(self.health_watchdog.LIVELINESS)
        self.assertEqual(health.id_, "test_dispenser")
        self.assertEqual(health.health_status, ttm.HealthStatus.DEAD)

    async def _check_dispenser_mode(self, mode: int, expected_health: ttm.HealthStatus):
        health: Optional[ttm.RobotHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.dispenser_health.subscribe(assign)
        state = test_data.make_dispenser_state()
        state.mode = mode
        self.rmf.dispenser_states.on_next(state)
        self.assertEqual(health.health_status, expected_health)

    async def test_dispenser_mode(self):
        test_cases = [
            (DispenserState.IDLE, ttm.HealthStatus.HEALTHY),
            (DispenserState.BUSY, ttm.HealthStatus.HEALTHY),
            (DispenserState.OFFLINE, ttm.HealthStatus.UNHEALTHY),
            (128, ttm.HealthStatus.UNHEALTHY),
        ]
        for test in test_cases:
            await self._check_dispenser_mode(*test)


class TestHealthWatchdog_IngestorHealth(BaseHealthWatchdogTests):
    async def test_heartbeat(self):
        await test_heartbeat(
            self,
            self.rmf.ingestor_health,
            self.rmf.ingestor_states,
            test_data.make_ingestor_state,
        )

    async def test_heartbeat_with_no_state(self):
        # simulate the situation where a ingestor state loaded from persistent
        # store never send any states.
        self.rmf.ingestor_states.on_next(test_data.make_ingestor_state("test_ingestor"))
        self.health_watchdog = HealthWatchdog(
            self.rmf, scheduler=self.scheduler, logger=self.logger
        )

        health: Optional[ttm.IngestorHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.ingestor_health.subscribe(assign)

        self.scheduler.advance_by(self.health_watchdog.LIVELINESS)
        self.assertEqual(health.id_, "test_ingestor")
        self.assertEqual(health.health_status, ttm.HealthStatus.DEAD)

    async def _check_ingestor_mode(self, mode: int, expected_health: ttm.HealthStatus):
        health: Optional[ttm.RobotHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.ingestor_health.subscribe(assign)
        state = test_data.make_ingestor_state()
        state.mode = mode
        self.rmf.ingestor_states.on_next(state)
        self.assertEqual(health.health_status, expected_health)

    async def test_ingestor_mode(self):
        test_cases = [
            (IngestorState.IDLE, ttm.HealthStatus.HEALTHY),
            (IngestorState.BUSY, ttm.HealthStatus.HEALTHY),
            (IngestorState.OFFLINE, ttm.HealthStatus.UNHEALTHY),
            (128, ttm.HealthStatus.UNHEALTHY),
        ]
        for test in test_cases:
            await self._check_ingestor_mode(*test)


class TestHealthWatchdog_RobotHealth(BaseHealthWatchdogTests):
    async def test_heartbeat(self):
        health = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.robot_health.subscribe(assign)

        def factory():
            state = test_data.make_fleet_state("test_fleet")
            state.robots = [test_data.make_robot_state("test_robot")]
            return state

        robot_id = ttm.get_robot_id("test_fleet", "test_robot")
        self.rmf.fleet_states.on_next(factory())
        self.scheduler.advance_by(0)
        self.assertEqual(health.health_status, ttm.HealthStatus.HEALTHY)
        self.assertEqual(health.id_, robot_id)

        # it should not be dead yet
        self.scheduler.advance_by(HealthWatchdog.LIVELINESS / 2)
        self.assertEqual(health.health_status, ttm.HealthStatus.HEALTHY)
        self.assertEqual(health.id_, robot_id)

        # # it should be dead now because the time between states has reached the threshold
        self.scheduler.advance_by(HealthWatchdog.LIVELINESS / 2)
        self.assertEqual(health.health_status, ttm.HealthStatus.DEAD)
        self.assertEqual(health.id_, robot_id)

        # # it should become alive again when a new state is emitted
        self.rmf.fleet_states.on_next(factory())
        self.assertEqual(health.health_status, ttm.HealthStatus.HEALTHY)
        self.assertEqual(health.id_, robot_id)

    async def test_heartbeat_with_no_state(self):
        # simulate the situation where a fleet state loaded from persistent
        # store never send any states.
        def factory():
            state = test_data.make_fleet_state("test_fleet")
            state.robots = [test_data.make_robot_state("test_robot")]
            return state

        self.rmf.fleet_states.on_next(factory())
        self.health_watchdog = HealthWatchdog(
            self.rmf, scheduler=self.scheduler, logger=self.logger
        )

        health: Optional[ttm.RobotHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.robot_health.subscribe(assign)

        robot_id = ttm.get_robot_id("test_fleet", "test_robot")
        self.scheduler.advance_by(self.health_watchdog.LIVELINESS)
        self.assertEqual(health.id_, robot_id)
        self.assertEqual(health.health_status, ttm.HealthStatus.DEAD)

    async def _check_robot_mode(self, mode: int, expected_health: ttm.HealthStatus):
        health: Optional[ttm.RobotHealth] = None

        def assign(v):
            nonlocal health
            health = v

        self.rmf.robot_health.subscribe(assign)
        state = test_data.make_robot_state()
        state.mode.mode = mode
        fleet_state = test_data.make_fleet_state()
        fleet_state.robots = [state]
        self.rmf.fleet_states.on_next(fleet_state)
        self.assertEqual(health.health_status, expected_health)

    async def test_robot_mode(self):
        test_cases = [
            (RobotMode.MODE_IDLE, ttm.HealthStatus.HEALTHY),
            (RobotMode.MODE_CHARGING, ttm.HealthStatus.HEALTHY),
            (RobotMode.MODE_MOVING, ttm.HealthStatus.HEALTHY),
            (RobotMode.MODE_PAUSED, ttm.HealthStatus.HEALTHY),
            (RobotMode.MODE_WAITING, ttm.HealthStatus.HEALTHY),
            (RobotMode.MODE_GOING_HOME, ttm.HealthStatus.HEALTHY),
            (RobotMode.MODE_DOCKING, ttm.HealthStatus.HEALTHY),
            (RobotMode.MODE_EMERGENCY, ttm.HealthStatus.UNHEALTHY),
            (RobotMode.MODE_ADAPTER_ERROR, ttm.HealthStatus.UNHEALTHY),
            (128, ttm.HealthStatus.UNHEALTHY),
        ]
        for test in test_cases:
            await self._check_robot_mode(*test)
