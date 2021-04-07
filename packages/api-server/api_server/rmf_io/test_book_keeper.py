import asyncio
import logging
import unittest

from rmf_door_msgs.msg import DoorMode
from rmf_task_msgs.msg import TaskSummary
from tortoise import Tortoise

from ..models import tortoise_models as ttm
from . import test_data
from .book_keeper import RmfBookKeeper
from .gateway import RmfGateway


class TestRmfBookKeeper(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await Tortoise.init(
            db_url="sqlite://:memory:",
            modules={"models": ["api_server.models.tortoise_models"]},
        )
        await Tortoise.generate_schemas()
        self.rmf = RmfGateway()
        logger = logging.Logger("test", level="CRITICAL")
        self.book_keeper = RmfBookKeeper(self.rmf, logger=logger)
        self.book_keeper.start()

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_write_door_state(self):
        state = test_data.make_door_state("test_door")
        state.current_mode.value = DoorMode.MODE_OPEN
        self.rmf.door_states.on_next(state)
        await asyncio.sleep(0)
        result = await ttm.DoorState.get(id_="test_door")
        self.assertIsNotNone(result)
        result_rmf = result.to_rmf()
        self.assertEqual(result_rmf.current_mode.value, DoorMode.MODE_OPEN)

        state = test_data.make_door_state("test_door")
        state.current_mode.value = DoorMode.MODE_CLOSED
        self.rmf.door_states.on_next(state)
        await asyncio.sleep(0)
        result = await ttm.DoorState.get(id_="test_door")
        self.assertIsNotNone(result)
        result_rmf = result.to_rmf()
        self.assertEqual(result_rmf.current_mode.value, DoorMode.MODE_CLOSED)

    async def test_write_door_health(self):
        self.rmf.door_health.on_next(
            ttm.DoorHealth(
                id_="test_door",
                health_status=ttm.HealthStatus.HEALTHY,
            )
        )
        await asyncio.sleep(0)
        health = await ttm.DoorHealth.get(id_="test_door")
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, ttm.HealthStatus.HEALTHY)

        self.rmf.door_health.on_next(
            ttm.DoorHealth(
                id_="test_door",
                health_status=ttm.HealthStatus.UNHEALTHY,
            )
        )
        await asyncio.sleep(0)
        health = await ttm.DoorHealth.get(id_="test_door")
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, ttm.HealthStatus.UNHEALTHY)

    async def test_write_lift_state(self):
        state = test_data.make_lift_state("test_lift")
        state.available_floors = ["L1", "L2"]
        state.current_floor = "L1"
        self.rmf.lift_states.on_next(state)
        await asyncio.sleep(0)
        result = await ttm.LiftState.get(id_="test_lift")
        self.assertIsNotNone(result)
        result_rmf = result.to_rmf()
        self.assertEqual(result_rmf.current_floor, "L1")

        state = test_data.make_lift_state("test_lift")
        state.available_floors = ["L1", "L2"]
        state.current_floor = "L2"
        self.rmf.lift_states.on_next(state)
        await asyncio.sleep(0)
        result = await ttm.LiftState.get(id_="test_lift")
        self.assertIsNotNone(result)
        result_rmf = result.to_rmf()
        self.assertEqual(result_rmf.current_floor, "L2")

    async def test_write_lift_health(self):
        self.rmf.lift_health.on_next(
            ttm.LiftHealth(
                id_="test_lift",
                health_status=ttm.HealthStatus.HEALTHY,
            )
        )
        await asyncio.sleep(0)
        health = await ttm.LiftHealth.get(id_="test_lift")
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, ttm.HealthStatus.HEALTHY)

        self.rmf.lift_health.on_next(
            ttm.LiftHealth(
                id_="test_lift",
                health_status=ttm.HealthStatus.UNHEALTHY,
            )
        )
        await asyncio.sleep(0)
        health = await ttm.LiftHealth.get(id_="test_lift")
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, ttm.HealthStatus.UNHEALTHY)

    async def test_write_dispenser_state(self):
        state = test_data.make_dispenser_state("test_dispenser")
        state.seconds_remaining = 2.0
        self.rmf.dispenser_states.on_next(state)
        await asyncio.sleep(0)
        result = await ttm.DispenserState.get(id_="test_dispenser")
        self.assertIsNotNone(result)
        result_rmf = result.to_rmf()
        self.assertEqual(result_rmf.seconds_remaining, 2.0)

        state = test_data.make_dispenser_state("test_dispenser")
        state.seconds_remaining = 1.0
        self.rmf.dispenser_states.on_next(state)
        await asyncio.sleep(0)
        result = await ttm.DispenserState.get(id_="test_dispenser")
        self.assertIsNotNone(result)
        result_rmf = result.to_rmf()
        self.assertEqual(result_rmf.seconds_remaining, 1.0)

    async def test_write_dispenser_health(self):
        self.rmf.dispenser_health.on_next(
            ttm.DispenserHealth(
                id_="test_dispenser",
                health_status=ttm.HealthStatus.HEALTHY,
            )
        )
        await asyncio.sleep(0)
        health = await ttm.DispenserHealth.get(id_="test_dispenser")
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, ttm.HealthStatus.HEALTHY)

        self.rmf.dispenser_health.on_next(
            ttm.DispenserHealth(
                id_="test_dispenser",
                health_status=ttm.HealthStatus.UNHEALTHY,
            )
        )
        await asyncio.sleep(0)
        health = await ttm.DispenserHealth.get(id_="test_dispenser")
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, ttm.HealthStatus.UNHEALTHY)

    async def test_write_ingestor_health(self):
        self.rmf.ingestor_health.on_next(
            ttm.IngestorHealth(
                id_="test_ingestor",
                health_status=ttm.HealthStatus.HEALTHY,
            )
        )
        await asyncio.sleep(0)
        health = await ttm.IngestorHealth.get(id_="test_ingestor")
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, ttm.HealthStatus.HEALTHY)

        self.rmf.ingestor_health.on_next(
            ttm.IngestorHealth(
                id_="test_ingestor",
                health_status=ttm.HealthStatus.UNHEALTHY,
            )
        )
        await asyncio.sleep(0)
        health = await ttm.IngestorHealth.get(id_="test_ingestor")
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, ttm.HealthStatus.UNHEALTHY)

    async def test_write_fleet_state(self):
        state = test_data.make_fleet_state("test_fleet")
        state.robots = [test_data.make_robot_state()]
        self.rmf.fleet_states.on_next(state)
        await asyncio.sleep(0)
        result = await ttm.FleetState.get(id_="test_fleet")
        self.assertIsNotNone(result)
        result_rmf = result.to_rmf()
        self.assertEqual(len(result_rmf.robots), 1)

        state = test_data.make_fleet_state("test_fleet")
        state.robots = []
        self.rmf.fleet_states.on_next(state)
        await asyncio.sleep(0)
        result = await ttm.FleetState.get(id_="test_fleet")
        self.assertIsNotNone(result)
        result_rmf = result.to_rmf()
        self.assertEqual(len(result_rmf.robots), 0)

    async def test_write_robot_health(self):
        self.rmf.robot_health.on_next(
            ttm.RobotHealth(
                id_="test_robot",
                health_status=ttm.HealthStatus.HEALTHY,
            )
        )
        await asyncio.sleep(0)
        health = await ttm.RobotHealth.get(id_="test_robot")
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, ttm.HealthStatus.HEALTHY)

        self.rmf.robot_health.on_next(
            ttm.RobotHealth(
                id_="test_robot",
                health_status=ttm.HealthStatus.UNHEALTHY,
            )
        )
        await asyncio.sleep(0)
        health = await ttm.RobotHealth.get(id_="test_robot")
        self.assertIsNotNone(health)
        self.assertEqual(health.health_status, ttm.HealthStatus.UNHEALTHY)

    async def test_write_task_summary(self):
        task = TaskSummary(task_id="test_task")
        task.status = "test_status"
        self.rmf.task_summaries.on_next(task)
        await asyncio.sleep(0)
        result = await ttm.TaskSummary.get(id_="test_task")
        self.assertIsNotNone(result)
        result_rmf = result.to_rmf()
        self.assertEqual(result_rmf.status, "test_status")

        task = TaskSummary(task_id="test_task")
        task.status = "test_status_2"
        self.rmf.task_summaries.on_next(task)
        await asyncio.sleep(0)
        result = await ttm.TaskSummary.get(id_="test_task")
        self.assertIsNotNone(result)
        result_rmf = result.to_rmf()
        self.assertEqual(result_rmf.status, "test_status_2")
