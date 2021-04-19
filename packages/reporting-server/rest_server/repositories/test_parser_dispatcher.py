import unittest

# Husky is sorting tortoise in a way that tortoise goes after our custom packages.
# and because of that the lint is failing
import tortoise
from fastapi.testclient import TestClient
from models import (
    DispenserState,
    DoorState,
    FleetState,
    HealthStatus,
    IngestorState,
    LiftState,
)
from rest_server.app import app

from .parser_dispacher import log_model_dispacher

# Example of logs
# {'log': 'INFO:app.BookKeeper.dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n', 'stream': 'stdout'}

# {'log': 'INFO:app.BookKeeper.door_state:{"door_time": {"sec": 1596, "nanosec": 548000000}, "door_name": "hardware_door", "current_mode": {"value": 0}}\n', 'stream': 'stdout'}

# 'INFO:app.BookKeeper.fleet_state:{"name": "tinyRobot", "robots": [{"name": "tinyRobot1", "model": "", "task_id": "", "seq": 3194, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1600, "nanosec": 189000000}, "x": 11.55367374420166, "y": -11.317498207092285, "yaw": -1.5998055934906006, "level_name": "L1", "index": 0}, "path": []}, {"name": "tinyRobot2", "model": "", "task_id": "", "seq": 3194, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1600, "nanosec": 189000000}, "x": 15.15751838684082, "y": -11.22861385345459, "yaw": -1.5839799642562866, "level_name": "L1", "index": 0}, "path": []}]}\n', 'stream': 'stdout'}

# {'log': 'INFO:app.BookKeeper.door_health:{"id": "hardware_door", "health_status": "HealthStatus.HEALTHY", "health_message": null}\n', 'stream': 'stdout'}

# {'log': 'INFO:app.BookKeeper.ingestor_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_ingestor", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n', 'stream': 'stdout'}

# {'log': 'INFO:app.BookKeeper.lift_state:{"time": {"sec": 1600, "nanosec": 0}, "state": 1, "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n', 'stream': 'stdout'}


class TestCaseLogParserDispatcher(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await tortoise.Tortoise.init(
            db_url="sqlite://:memory:",
            modules={"models": ["models"]},
        )
        await tortoise.Tortoise.generate_schemas()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await tortoise.Tortoise.close_connections()

    async def test_dispenser_state_created(self):
        data = 'dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n'
        await log_model_dispacher(data)
        instance = await DispenserState.first()
        self.assertEqual(instance.guid, "coke_dispenser")

    async def test_door_state_created(self):
        data = 'door_state:{"door_time": {"sec": 1596, "nanosec": 548000000}, "door_name": "hardware_door", "current_mode": {"value": 0}}\n'
        await log_model_dispacher(data)
        instance = await DoorState.first()
        self.assertEqual(instance.name, "hardware_door")

    async def test_fleet_state_created(self):
        data = 'fleet_state:{"name": "tinyRobot", "robots": [{"name": "tinyRobot1", "model": "", "task_id": "", "seq": 3194, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1600, "nanosec": 189000000}, "x": 11.55367374420166, "y": -11.317498207092285, "yaw": -1.5998055934906006, "level_name": "L1", "index": 0}, "path": []}, {"name": "tinyRobot2", "model": "", "task_id": "", "seq": 3194, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1600, "nanosec": 189000000}, "x": 15.15751838684082, "y": -11.22861385345459, "yaw": -1.5839799642562866, "level_name": "L1", "index": 0}, "path": []}]}\n'
        await log_model_dispacher(data)
        instance = await FleetState.all()
        self.assertEqual(len(instance), 2)

        self.assertEqual(instance[0].fleet_name, "tinyRobot")
        self.assertEqual(instance[0].robot_name, "tinyRobot1")

        self.assertEqual(instance[1].fleet_name, "tinyRobot")
        self.assertEqual(instance[1].robot_name, "tinyRobot2")

    async def test_door_health_created(self):
        data = 'door_health:{"id": "hardware_door", "health_status": "HealthStatus.HEALTHY", "health_message": null}\n'
        await log_model_dispacher(data)
        instance = await HealthStatus.first()
        self.assertEqual(instance.actor_id, "hardware_door")
        self.assertEqual(instance.device, "door_health")

    async def test_robot_health_created(self):
        data = 'robot_health:{"id": "robot1", "health_status": "HealthStatus.HEALTHY", "health_message": null}\n'
        await log_model_dispacher(data)
        instance = await HealthStatus.first()
        self.assertEqual(instance.actor_id, "robot1")
        self.assertEqual(instance.device, "robot_health")

    async def test_lift_health_created(self):
        data = 'lift_health:{"id": "lift1", "health_status": "HealthStatus.HEALTHY", "health_message": null}\n'
        await log_model_dispacher(data)
        instance = await HealthStatus.first()
        self.assertEqual(instance.actor_id, "lift1")
        self.assertEqual(instance.device, "lift_health")

    async def test_ingestor_state_created(self):
        data = 'ingestor_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_ingestor", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n'
        await log_model_dispacher(data)
        instance = await IngestorState.first()
        self.assertEqual(instance.guid, "coke_ingestor")

    async def test_lift_state_created(self):
        data = 'lift_state: {"lift_name": "test_lift", "lift_time": 0, "available_floors": ["L1", "L2"], "current_floor": "L1", "destination_floor": "L2", "door_state": 0, "motion_state": 0, "available_modes": [0], "current_mode": 0, "session_id": "test_session"}\n'
        await log_model_dispacher(data)
        instance = await LiftState.first()
        self.assertEqual(instance.name, "test_lift")
