import unittest
from datetime import datetime, timedelta

from fastapi.testclient import TestClient
from tortoise import Tortoise

from task_scheduler.app import get_app
from task_scheduler.models.tortoise_models import ScheduledTask, TaskRule, TaskTypeEnum
from task_scheduler.models.tortoise_models.helpers.task_rule_definition import (
    FrequencyEnum,
)
from task_scheduler.repositories.scheduled_task_handler import ScheduledTaskRepository
from task_scheduler.test_utils import start_test_database

app = get_app()


class TestCaseScheduledTasks(unittest.IsolatedAsyncioTestCase):
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

        self.task_rule2 = await TaskRule.create(
            name="test2",
            task_type=TaskTypeEnum.DELIVERY,
            frequency=1,
            frequency_type=FrequencyEnum.ONCE,
            first_day_to_apply_rule=now,
            start_datetime=now,
        )

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    # Given
    # scheduled_task_id = 1
    # # When
    # response = await self.client.delete(
    #     f"/scheduled_task/{scheduled_task_id}"
    # )
    # # Then
    # assert response.status_code == 200

    async def test_deletes_scheduled_task(self):
        await ScheduledTask.create(
            task_rule=self.task_rule,
            task_type=TaskTypeEnum.LOOP,
            task_datetime=datetime.utcnow(),
        )
        tasks = await ScheduledTask.all()
        self.assertEqual(len(tasks), 3)

        await ScheduledTaskRepository.delete(tasks[1].id)
        self.assertEqual(len(await ScheduledTask.all()), 2)

    async def test_fails_to_delete_inexistent_scheduled_task(self):
        with self.assertRaises(Exception):
            await ScheduledTaskRepository.delete(999)

    async def test_gets_scheduled_tasks(self):
        await ScheduledTask.create(
            task_rule=self.task_rule,
            task_type=TaskTypeEnum.LOOP,
            task_datetime=datetime.utcnow(),
        )
        await ScheduledTask.create(
            task_rule=self.task_rule2,
            task_type=TaskTypeEnum.LOOP,
            task_datetime=datetime.utcnow(),
        )
        tasks = await ScheduledTaskRepository.get(0, 500)
        self.assertEqual(len(tasks), 4)

    async def test_gets_and_filters_scheduled_tasks(self):
        now = datetime.utcnow()
        rule = await TaskRule.create(
            name="test3",
            task_type=TaskTypeEnum.DELIVERY,
            frequency=1,
            frequency_type=FrequencyEnum.DAILY,
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=now + timedelta(days=2),
        )

        self.assertEqual(len(await ScheduledTask.all()), 5)
        tasks1 = await ScheduledTaskRepository.get(0, 500, self.task_rule.id)
        self.assertEqual(len(tasks1), 1)

        tasks2 = await ScheduledTaskRepository.get(0, 500, self.task_rule2.id)
        self.assertEqual(len(tasks2), 1)

        tasks3 = await ScheduledTaskRepository.get(0, 500, rule.id)
        self.assertEqual(len(tasks3), 3)
