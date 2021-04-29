# conflicts with isort because of local non-relative import
# pylint: disable=wrong-import-order
import unittest

from fastapi.testclient import TestClient
from models import DispenserState, DoorState, RawLog
from rest_server.app import get_app
from tortoise import Tortoise

from .log_creation_handler import create_raw_log, create_rmf_server_log

app = get_app()


class TestCaseLogRMFServerCreationRepository(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await Tortoise.init(
            db_url="sqlite://:memory:",
            modules={"models": ["models"]},
        )
        await Tortoise.generate_schemas()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_no_data_sent_to_rmfserver(self):
        response = await create_rmf_server_log([])
        self.assertEqual(response, "No data received")

    async def test_create_a_rmfserver_log_correctly(self):
        data = [
            {
                "log": 'INFO:app.BookKeeper.dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n',
                "stream": "stdout",
            },
            {
                "log": 'INFO:app.BookKeeper.door_state:{"door_time": {"sec": 1596, "nanosec": 548000000}, "door_name": "hardware_door", "current_mode": {"value": 0}}\n',
                "stream": "stdout",
            },
        ]

        response = await create_rmf_server_log(data)
        self.assertEqual(response, "Logs were saved correctly")

    async def test_rmfserver_handle_and_return_error(self):
        data = [
            {
                "log2": 'INFO:app.BookKeeper.dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n',
                "stream": "stdout",
            },
            {
                "log": 'INFO:app.BookKeeper.door_state:{"door_time": {"sec": 1596, "nanosec": 548000000}, "door_name": "hardware_door", "current_mode": {"value": 0}}\n',
                "stream": "stdout",
            },
        ]

        response = await create_rmf_server_log(data)
        self.assertEqual(len(response), 1)

    async def test_rmfserver_handle_creation_of_logs(self):
        data = [
            {
                "log": 'INFO:app.BookKeeper.dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n',
                "stream": "stdout",
            },
            {
                "log": 'INFO:app.BookKeeper.door_state:{"door_time": {"sec": 1596, "nanosec": 548000000}, "door_name": "hardware_door", "current_mode": {"value": 0}}\n',
                "stream": "stdout",
            },
        ]

        await create_rmf_server_log(data)
        dispenser = await DispenserState.first()
        door = await DoorState.first()
        self.assertEqual(dispenser.guid, "coke_dispenser")
        self.assertEqual(door.name, "hardware_door")


class TestCaseRawLogCreationRepository(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await Tortoise.init(
            db_url="sqlite://:memory:",
            modules={"models": ["models"]},
        )
        await Tortoise.generate_schemas()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_no_data_sent_to_rawlog(self):
        response = await create_raw_log([])
        self.assertEqual(response, "No data received")

    async def test_create_a_raw_log_correctly(self):
        data = [
            {
                "log": 'INFO:app.BookKeeper.dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n',
                "stream": "stdout",
            },
            {
                'INFO:app.BookKeeper.door_state:{"door_time": {"sec": 1596, "nanosec": 548000000}, "door_name": "hardware_door", "current_mode": {"value": 0}}\n'
            },
            "this is a test",
        ]

        response = await create_raw_log(data)
        self.assertEqual(response, "Logs were saved correctly")

    async def test_raw_log_handle_creation_of_logs(self):
        data = [
            {
                "log": 'INFO:app.BookKeeper.dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n',
                "stream": "stdout",
            },
            {
                "log": 'INFO:app.BookKeeper.door_state:{"door_time": {"sec": 1596, "nanosec": 548000000}, "door_name": "hardware_door", "current_mode": {"value": 0}}\n',
                "stream": "stdout",
            },
            "this is a test",
        ]

        await create_raw_log(data)
        dispenser = await RawLog.all()
        self.assertEqual(len(dispenser), 3)
