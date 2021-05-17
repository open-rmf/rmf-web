# conflicts with isort because of local non-relative import
# pylint: disable=wrong-import-order
import unittest

from fastapi.testclient import TestClient
from models import DispenserState, DoorState, RawLog
from models.auth_events import AuthEvents
from rest_server.app import get_app
from tortoise import Tortoise

from .log_creation_handler import (
    create_keycloak_log,
    create_raw_log,
    create_rmf_server_log,
)

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

    async def test_raw_log_handle_creation_of_logs_with_container_name(self):
        data = [
            {
                "log": 'INFO:app.BookKeeper.dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n',
                "stream": "stdout",
                "kubernetes": {
                    "container_name": "app-that-writes-logs",
                    "namespace_name": "default",
                    "pod_name": "app-that-writes-logs",
                    "container_image": "busybox:latest",
                    "container_image_id": "docker-pullable://busybox@sha256:ae39a6f5c07297d7ab64dbd4f82c77c874cc6a94cea29fdec309d0992574b4f7",
                    "pod_id": "978761c6-2a19-422f-b710-d43da2348f1f",
                    "host": "minikube",
                    "master_url": "https://10.96.0.1:443/api",
                    "namespace_id": "e192acd4-e6e7-46c2-8514-44a27a367749",
                },
            },
        ]

        await create_raw_log(data)
        log = await RawLog.first()
        self.assertEqual(log.container_name, "app-that-writes-logs")

    async def test_keacloak_log_creation(self):
        data = [
            {
                "log": '[0m[0m20:41:54,721 INFO  [org.keycloak.events] (default task-2) JSON_EVENT::{"type":"LOGIN_ERROR","realmId":"579ce396-83c7-4094-964d-7ea07553089f","clientId":"reporting","ipAddress":"192.168.49.1","error":"user_not_found","auth_method":"openid-connect","auth_type":"code","redirect_uri":"https://example.com/reporting","code_id":"f813403c-2732-4062-9911-cf65b89a2278","username":"test"}',
                "stream": "stdout",
                "kubernetes": {
                    "container_name": "app-that-writes-logs",
                },
            },
            {
                "log": '19:47:08,004 INFO  [org.keycloak.events] (default task-3) JSON_EVENT::{"type":"LOGIN","realmId":"master","clientId":"security-admin-console","userId":"7d2f3cdd-9778-4847-ab9d-db68f70f043f","ipAddress":"172.22.0.1","auth_method":"openid-connect","auth_type":"code","redirect_uri":"http://localhost:8080/auth/admin/master/console/","consent":"no_consent_required","code_id":"ac8c82d7-45ac-4227-86d3-e167b176e26f","username":"admin"}',
                "stream": "stdout",
            },
        ]

        await create_keycloak_log(data)
        logs = await AuthEvents.all()
        self.assertEqual(len(logs), 2)

    async def test_keacloak_logout_creation(self):
        data = [
            {
                "log": '19:47:20,649 INFO  [org.keycloak.events] (default task-6) JSON_EVENT::{"type":"LOGOUT","realmId":"master","userId":"7d2f3cdd-9778-4847-ab9d-db68f70f043f","ipAddress":"172.22.0.1","redirect_uri":"http://localhost:8080/auth/admin/master/console/#/realms/master"}',
                "stream": "stdout",
            },
        ]

        await create_keycloak_log(data)
        logs = await AuthEvents.all()
        self.assertEqual(len(logs), 1)
