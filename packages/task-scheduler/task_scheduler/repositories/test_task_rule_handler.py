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

    async def test_deletes_scheduled_task(self):

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

        tasks = await TaskRule.all()
        self.assertEqual(len(tasks), 1)

        await TaskRuleRepository.delete(tasks[0].id)
        self.assertEqual(len(await ScheduledTask.all()), 0)

    # async def test_gets_scheduled_tasks(self):
    #     await ScheduledTask.create(
    #         task_rule=self.task_rule,
    #         task_type=TaskTypeEnum.LOOP,
    #         task_datetime=datetime.utcnow(),
    #     )
    #     await ScheduledTask.create(
    #         task_rule=self.task_rule2,
    #         task_type=TaskTypeEnum.LOOP,
    #         task_datetime=datetime.utcnow(),
    #     )
    #     tasks = await ScheduledTaskRepository.get(0, 500)
    #     self.assertEqual(len(tasks), 4)

    # async def test_gets_and_filters_scheduled_tasks(self):
    #     now = datetime.utcnow()
    #     rule = await TaskRule.create(
    #         description="test3",
    #         task_type=TaskTypeEnum.DELIVERY,
    #         frequency=1,
    #         frequency_type=FrequencyEnum.DAILY,
    #         first_day_to_apply_rule=now,
    #         start_datetime=now,
    #         end_datetime=now + timedelta(days=2)
    #     )

    #     self.assertEqual(len(await ScheduledTask.all()), 5)
    #     tasks1 = await ScheduledTaskRepository.get(0, 500, self.task_rule.id)
    #     self.assertEqual(len(tasks1), 1)

    #     tasks2 = await ScheduledTaskRepository.get(0, 500, self.task_rule2.id)
    #     self.assertEqual(len(tasks2), 1)

    #     tasks3 = await ScheduledTaskRepository.get(0, 500, rule.id)
    #     self.assertEqual(len(tasks3), 3)
