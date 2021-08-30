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


class TestScheduledTaskRouter(unittest.IsolatedAsyncioTestCase):
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

    def test_deletes_scheduled_task(self):
        response = self.client.delete("/task/scheduled/1")
        assert response.status_code == 204

    def test_fails_on_deleting_scheduled_task(self):
        response = self.client.delete("/task/scheduled/100")
        assert response.status_code == 503

    def test_gets_scheduled_task(self):
        response = self.client.get("/task/scheduled/")
        assert response.status_code == 200
        self.assertEqual(len(response.json()), 1)
