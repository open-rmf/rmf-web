import unittest
from datetime import datetime

from fastapi.testclient import TestClient
from tortoise import Tortoise

from task_scheduler.app import get_app
from task_scheduler.models.tortoise_models import ScheduledTask, TaskRule, TaskTypeEnum
from task_scheduler.models.tortoise_models.helpers.task_rule_definition import (
    FrequencyEnum,
)
from task_scheduler.repositories.task_rule_handler import TaskRuleRepository
from task_scheduler.test_utils import start_test_database

app = get_app()


class TestCaseTaskRule(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await start_test_database()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_creates_task_rule_without_weekdays(self):

        now = datetime.utcnow()
        task_rule = await TaskRuleRepository.create(
            {
                "description": "test",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": now,
            }
        )

        self.assertEqual(task_rule.description, "test")

    async def test_fails_on_creation_without_enddate_and_frequency_daily(self):
        with self.assertRaises(Exception):
            now = datetime.utcnow()
            await TaskRuleRepository.create(
                {
                    "description": "test",
                    "task_type": "delivery",
                    "frequency": 1,
                    "frequency_type": "Daily",
                    "start_datetime": now,
                }
            )

    async def test_creates_task_rule_with_weekdays(self):

        now = datetime.utcnow()
        task_rule = await TaskRuleRepository.create(
            {
                "description": "test",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": now,
                "days_of_week": [True, True, True, True, True, False, False],
            }
        )

        self.assertEqual(task_rule.description, "test")

    async def test_deletes_task_rule(self):

        now = datetime.utcnow()
        await TaskRuleRepository.create(
            {
                "description": "test",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": now,
                "days_of_week": [True, True, True, True, True, False, False],
            }
        )

        tasks = await TaskRule.all()
        self.assertEqual(len(tasks), 1)

        await TaskRuleRepository.delete(tasks[0].id)
        self.assertEqual(len(await ScheduledTask.all()), 0)

    async def test_gets_task_rules(self):
        now = datetime.utcnow()

        await TaskRuleRepository.create(
            {
                "description": "test",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": now,
                "days_of_week": [True, True, True, True, True, False, False],
            }
        )

        await TaskRuleRepository.create(
            {
                "description": "test2",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": now,
                "days_of_week": [True, True, True, True, True, False, False],
            }
        )

        rules = await TaskRuleRepository.get(0, 500)
        self.assertEqual(len(rules), 2)

        rules2 = await TaskRuleRepository.get(0, 1)
        self.assertEqual(len(rules2), 1)
