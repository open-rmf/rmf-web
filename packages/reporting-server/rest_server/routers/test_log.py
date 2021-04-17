import unittest

from fastapi.testclient import TestClient
from tortoise import Tortoise

from ..app import app


class TestLogRoute(unittest.IsolatedAsyncioTestCase):
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

    # def test_error_on_bad_body(self):
    #     self.client = TestClient(app)
    #     response = self.client.post(
    #         "/log/all/",
    #         json=[{"log343": "test"}],
    #     )
    #     assert response.status_code == 503
