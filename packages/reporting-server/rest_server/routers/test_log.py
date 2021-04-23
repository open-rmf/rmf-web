import unittest

from fastapi.testclient import TestClient
from tortoise import Tortoise

from ..app import app


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
