import logging
import unittest

from rmf_door_msgs.msg import DoorMode
from tortoise import Tortoise

from api_server.models import (
    DispenserHealth,
    DispenserState,
    DoorHealth,
    DoorState,
    FleetState,
    HealthStatus,
    IngestorHealth,
    IngestorState,
    LiftHealth,
    LiftState,
    RobotHealth,
    TaskSummary,
)
from api_server.models import tortoise_models as ttm
from api_server.test import async_try_until, init_db, test_data

from .book_keeper import RmfBookKeeper
from .events import RmfEvents


class TestRmfBookKeeper(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await init_db()

        self.rmf = RmfEvents()
        logger = logging.Logger("test", level="CRITICAL")
        self.book_keeper = RmfBookKeeper(self.rmf, logger=logger)
        await self.book_keeper.start()

    async def asyncTearDown(self):
        await self.book_keeper.stop()
        await Tortoise.close_connections()

    async def test_write_door_state(self):
        state = test_data.make_door_state("test_door")
        state.current_mode.value = DoorMode.MODE_OPEN
        self.rmf.door_states.on_next(state)

        async def get():
            return DoorState.from_tortoise(await ttm.DoorState.get(id_="test_door"))

        result = await async_try_until(get, lambda x: x is not None, 1, 0.02)
        self.assertIsNotNone(result)
        self.assertEqual(result.current_mode.value, DoorMode.MODE_OPEN)

        state = test_data.make_door_state("test_door")
        state.current_mode.value = DoorMode.MODE_CLOSED
        self.rmf.door_states.on_next(state)
        result = await async_try_until(
            get, lambda x: x.current_mode.value == DoorMode.MODE_CLOSED, 1, 0.02
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.current_mode.value, DoorMode.MODE_CLOSED)

    async def test_write_door_health(self):
        self.rmf.door_health.on_next(
            DoorHealth(
                id_="test_door",
                health_status=HealthStatus.HEALTHY,
            )
        )

        async def get():
            return await DoorHealth.from_tortoise_orm(
                await ttm.DoorHealth.get(id_="test_door")
            )

        health = await async_try_until(get, lambda _: True, 1, 0.02)
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, HealthStatus.HEALTHY)

        self.rmf.door_health.on_next(
            DoorHealth(
                id_="test_door",
                health_status=HealthStatus.UNHEALTHY,
            )
        )
        health = await async_try_until(
            get, lambda x: x.health_status == HealthStatus.UNHEALTHY, 1, 0.02
        )
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, HealthStatus.UNHEALTHY)

    async def test_write_lift_state(self):
        state = test_data.make_lift_state("test_lift")
        state.available_floors = ["L1", "L2"]
        state.current_floor = "L1"
        self.rmf.lift_states.on_next(state)

        async def get():
            return LiftState.from_tortoise(await ttm.LiftState.get(id_="test_lift"))

        result = await async_try_until(get, lambda x: x is not None, 1, 0.02)
        self.assertIsNotNone(result)
        self.assertEqual(result.current_floor, "L1")

        state = test_data.make_lift_state("test_lift")
        state.available_floors = ["L1", "L2"]
        state.current_floor = "L2"
        self.rmf.lift_states.on_next(state)
        result = await async_try_until(get, lambda x: x.current_floor == "L2", 1, 0.02)
        self.assertIsNotNone(result)
        self.assertEqual(result.current_floor, "L2")

    async def test_write_lift_health(self):
        self.rmf.lift_health.on_next(
            LiftHealth(
                id_="test_lift",
                health_status=HealthStatus.HEALTHY,
            )
        )

        async def get():
            return await LiftHealth.from_tortoise_orm(
                await ttm.LiftHealth.get(id_="test_lift")
            )

        health = await async_try_until(get, lambda _: True, 1, 0.02)
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, HealthStatus.HEALTHY)

        self.rmf.lift_health.on_next(
            LiftHealth(
                id_="test_lift",
                health_status=HealthStatus.UNHEALTHY,
            )
        )
        health = await async_try_until(
            get, lambda x: x.health_status == HealthStatus.UNHEALTHY, 1, 0.02
        )
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, HealthStatus.UNHEALTHY)

    async def test_write_dispenser_state(self):
        state = test_data.make_dispenser_state("test_dispenser")
        state.seconds_remaining = 2.0
        self.rmf.dispenser_states.on_next(state)

        async def get_state():
            return DispenserState.from_tortoise(
                await ttm.DispenserState.get(id_="test_dispenser")
            )

        result = await async_try_until(get_state, lambda x: x is not None, 1, 0.02)
        self.assertIsNotNone(result)
        self.assertEqual(result.seconds_remaining, 2.0)

        state = test_data.make_dispenser_state("test_dispenser")
        state.seconds_remaining = 1.0
        self.rmf.dispenser_states.on_next(state)
        result = await async_try_until(
            get_state, lambda x: x.seconds_remaining == 1.0, 1, 0.02
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.seconds_remaining, 1.0)

    async def test_write_dispenser_health(self):
        self.rmf.dispenser_health.on_next(
            DispenserHealth(
                id_="test_dispenser",
                health_status=HealthStatus.HEALTHY,
            )
        )

        async def get():
            return await DispenserHealth.from_tortoise_orm(
                await ttm.DispenserHealth.get(id_="test_dispenser")
            )

        health = await async_try_until(get, lambda _: True, 1, 0.02)
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, HealthStatus.HEALTHY)

        self.rmf.dispenser_health.on_next(
            DispenserHealth(
                id_="test_dispenser",
                health_status=HealthStatus.UNHEALTHY,
            )
        )
        health = await async_try_until(
            get, lambda x: x.health_status == HealthStatus.UNHEALTHY, 1, 0.02
        )
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, HealthStatus.UNHEALTHY)

    async def test_write_ingestor_state(self):
        state = test_data.make_ingestor_state("test_ingestor")
        state.seconds_remaining = 2.0
        self.rmf.ingestor_states.on_next(state)

        async def get():
            return IngestorState.from_tortoise(
                await ttm.IngestorState.get(id_="test_ingestor")
            )

        result = await async_try_until(get, lambda x: x is not None, 1, 0.02)
        self.assertIsNotNone(result)
        self.assertEqual(result.seconds_remaining, 2.0)

        state = test_data.make_ingestor_state("test_ingestor")
        state.seconds_remaining = 1.0
        self.rmf.ingestor_states.on_next(state)
        result = await async_try_until(
            get, lambda x: x.seconds_remaining == 1.0, 1, 0.02
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.seconds_remaining, 1.0)

    async def test_write_ingestor_health(self):
        self.rmf.ingestor_health.on_next(
            IngestorHealth(
                id_="test_ingestor",
                health_status=HealthStatus.HEALTHY,
            )
        )

        async def get():
            return await IngestorHealth.from_tortoise_orm(
                await ttm.IngestorHealth.get(id_="test_ingestor")
            )

        health = await async_try_until(get, lambda _: True, 1, 0.02)
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, HealthStatus.HEALTHY)

        self.rmf.ingestor_health.on_next(
            IngestorHealth(
                id_="test_ingestor",
                health_status=HealthStatus.UNHEALTHY,
            )
        )
        health = await async_try_until(
            get, lambda x: x.health_status == HealthStatus.UNHEALTHY, 1, 0.02
        )
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, HealthStatus.UNHEALTHY)

    async def test_write_fleet_state(self):
        state = test_data.make_fleet_state("test_fleet")
        state.robots = [test_data.make_robot_state()]
        self.rmf.fleet_states.on_next(state)

        async def get():
            return FleetState.from_tortoise(await ttm.FleetState.get(id_="test_fleet"))

        result = await async_try_until(get, lambda _: True, 1, 0.02)
        self.assertIsNotNone(result)
        self.assertEqual(len(result.robots), 1)

        state = test_data.make_fleet_state("test_fleet")
        state.robots = []
        self.rmf.fleet_states.on_next(state)
        result = await async_try_until(get, lambda x: len(x.robots) == 0, 1, 0.02)
        self.assertIsNotNone(result)
        self.assertEqual(len(result.robots), 0)

    async def test_write_robot_health(self):
        self.rmf.robot_health.on_next(
            RobotHealth(
                id_="test_fleet/test_robot",
                health_status=HealthStatus.HEALTHY,
            )
        )

        async def get():
            return await RobotHealth.from_tortoise_orm(
                await ttm.RobotHealth.get(id_="test_fleet/test_robot")
            )

        health = await async_try_until(get, lambda _: True, 1, 0.02)
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, HealthStatus.HEALTHY)

        self.rmf.robot_health.on_next(
            RobotHealth(
                id_="test_fleet/test_robot",
                health_status=HealthStatus.UNHEALTHY,
            )
        )
        health = await async_try_until(
            get, lambda x: x.health_status == HealthStatus.UNHEALTHY, 1, 0.02
        )
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, HealthStatus.UNHEALTHY)

    async def test_write_task_summary(self):
        task = TaskSummary(task_id="test_task")
        task.status = "test_status"
        self.rmf.task_summaries.on_next(task)

        async def get():
            return TaskSummary.from_tortoise(await ttm.TaskSummary.get(id_="test_task"))

        result = await async_try_until(get, lambda _: True, 1, 0.02)
        self.assertIsNotNone(result)
        self.assertEqual(result.status, "test_status")

        task = TaskSummary(task_id="test_task")
        task.status = "test_status_2"
        self.rmf.task_summaries.on_next(task)
        result = await async_try_until(
            get, lambda x: x.status == "test_status_2", 1, 0.02
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.status, "test_status_2")
