import unittest
from datetime import datetime

from fastapi.testclient import TestClient
from tortoise import Tortoise

from task_scheduler.app import get_app
from task_scheduler.models.tortoise_models import ScheduledTask, TaskRule
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
        task_rule = await TaskRuleRepository.create(
            {
                "name": "test",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": "2021-08-23T15:11:03.979209",
            }
        )

        self.assertEqual(task_rule.name, "test")

    async def test_fails_on_creation_without_enddate_and_frequency_daily(self):
        with self.assertRaises(Exception):
            await TaskRuleRepository.create(
                {
                    "name": "test",
                    "task_type": "delivery",
                    "frequency": 1,
                    "frequency_type": "Daily",
                    "start_datetime": "2021-08-23T15:11:03.979209",
                }
            )

    async def test_creates_task_rule_with_weekdays(self):
        task_rule = await TaskRuleRepository.create(
            {
                "name": "test",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": "2021-08-23T15:11:03.979209",
                "days_of_week": [True, True, True, True, True, False, False],
            }
        )

        self.assertEqual(task_rule.name, "test")

    async def test_creates_task_rule_with_weekdays_empty(self):
        task_rule = await TaskRuleRepository.create(
            {
                "name": "test2",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": "2021-08-23T15:11:03.979209",
                "end_datetime": None,
                "days_of_week": [],
            }
        )

        self.assertEqual(task_rule.name, "test2")

    async def test_deletes_task_rule(self):

        await TaskRuleRepository.create(
            {
                "name": "test",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": "2021-08-23T15:11:03.979209",
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
                "name": "test",
                "task_type": "delivery",
                "frequency": 1,
                "frequency_type": "Once",
                "start_datetime": now,
                "days_of_week": [True, True, True, True, True, False, False],
            }
        )

        await TaskRuleRepository.create(
            {
                "name": "test2",
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
