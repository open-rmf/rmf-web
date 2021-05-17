import unittest

from fastapi.testclient import TestClient
from tortoise import Tortoise

from ..app import get_app

app = get_app()


class TestRawLogRoute(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await Tortoise.init(
            db_url="sqlite://:memory:",
            modules={"models": ["models"]},
        )
        await Tortoise.generate_schemas()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    def test_log_raw_creation(self):
        response = self.client.post(
            "/log/all/",
            json=[{"log": "test"}],
        )
        assert response.status_code == 201

    def test_error_on_bad_body_without_list(self):
        self.client = TestClient(app)
        response = self.client.post(
            "/log/all/",
            json={"log343": "test"},
        )
        assert response.status_code == 422


class TestRmfServerLogRoute(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await Tortoise.init(
            db_url="sqlite://:memory:",
            modules={"models": ["models"]},
        )
        await Tortoise.generate_schemas()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    def test_log_rmfserver_creation(self):
        response = self.client.post(
            "/log/rmfserver/",
            json=[{"log": "test"}],
        )
        assert response.status_code == 201

    def test_error_on_bad_body(self):
        self.client = TestClient(app)
        response = self.client.post(
            "/log/rmfserver/",
            json=[{"log343": "test"}],
        )
        assert response.status_code == 503

    def test_handle_error(self):
        self.client = TestClient(app)
        response = self.client.post(
            "/log/rmfserver/",
            json=[
                {"log343": "test"},
                {
                    "log": 'INFO:app.BookKeeper.dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n',
                    "stream": "stdout",
                },
            ],
        )

        assert response.status_code == 503


class TestKeycloakRoute(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await Tortoise.init(
            db_url="sqlite://:memory:",
            modules={"models": ["models"]},
        )
        await Tortoise.generate_schemas()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    def test_log_rmfserver_creation(self):
        response = self.client.post(
            "/log/keycloak/",
            json=[{"log": "test"}],
        )
        assert response.status_code == 201

    def test_error_on_bad_body(self):
        self.client = TestClient(app)
        response = self.client.post(
            "/log/keycloak/",
            json=[{"log343": "test"}],
        )
        assert response.status_code == 503

    def test_handle_error(self):
        self.client = TestClient(app)
        response = self.client.post(
            "/log/keycloak/",
            json=[
                {"log343": "test"},
                {
                    "log": '[0m[0m20:41:54,721 INFO  [org.keycloak.events] (default task-2) JSON_EVENT::{"type":"LOGIN_ERROR","realmId":"579ce396-83c7-4094-964d-7ea07553089f","clientId":"reporting","ipAddress":"192.168.49.1","error":"user_not_found","auth_method":"openid-connect","auth_type":"code","redirect_uri":"https://example.com/reporting","code_id":"f813403c-2732-4062-9911-cf65b89a2278","username":"test"}',
                    "stream": "stdout",
                },
            ],
        )

        assert response.status_code == 503
