# conflicts with isort because of local non-relative import
# pylint: disable=wrong-import-order

import unittest
from datetime import datetime

from fastapi.testclient import TestClient
from tortoise import Tortoise

from task_scheduler.models.tortoise_models import TaskRule, TaskTypeEnum
from task_scheduler.models.tortoise_models.helpers.task_rule_definition import (
    FrequencyEnum,
)
from task_scheduler.test_utils import start_test_database

from ..app import get_app

app = get_app()


class TestTaskRuleRouter(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await start_test_database()
        self.client = TestClient(app)

        now = datetime.utcnow()
        self.task_rule = await TaskRule.create(
            name="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.ONCE,
            first_day_to_apply_rule=now,
            start_datetime=now,
        )

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    def test_deletes_task_rule(self):
        response = self.client.delete("/task/rule/1")
        assert response.status_code == 204

    def test_fails_on_deleting_task_rule(self):
        response = self.client.delete("/task/rule/100")
        assert response.status_code == 503

    def test_gets_task_rule(self):
        response = self.client.get("/task/rule/")
        assert response.status_code == 200
        self.assertEqual(len(response.json()), 1)

    def test_creates_task_rule(self):
        response = self.client.post(
            "/task/rule/",
            json={
                "name": "test5",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": datetime.now().isoformat(),
                "days_of_week": [True, True, True, True, True, False, False],
            },
        )
        assert response.status_code == 201
        self.assertEqual(response.json()["name"], "test5")

    def test_creates_task_rule_with_args(self):
        response = self.client.post(
            "/task/rule/",
            json={
                "name": "test5",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": "2021-08-30T21:56:37.256Z",
                "end_datetime": None,
                "days_of_week": [],
                "args": {
                    "description": {"cleaning_zone": "lounge"},
                    "start_time": 1630360603,
                    "task_type": 4,
                    "priority": 0,
                },
            },
        )
        assert response.status_code == 201
        self.assertEqual(response.json()["name"], "test5")

    def test_fails_creating_wrong_rule_(self):
        response = self.client.post("/task/rule/", json={})
        assert response.status_code == 503
