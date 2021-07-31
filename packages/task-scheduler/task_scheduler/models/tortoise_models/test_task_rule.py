# conflicts with isort because of local non-relative import
# pylint: disable=wrong-import-order
import unittest
from datetime import datetime, timedelta

from fastapi.testclient import TestClient
from tortoise import Tortoise

from task_scheduler.app import get_app
from task_scheduler.test_utils import start_test_database

from .scheduled_task import ScheduledTask
from .task_rule import FrequencyEnum, TaskRule, TaskTypeEnum

app = get_app()


class TestCaseTaskRuleCreateEffect(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await start_test_database()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_create_task_rule_correctly(self):
        now = datetime.utcnow()
        task_rule = await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.ONCE,
            time_of_day=now,
            start_datetime=now,
        )

        self.assertEqual(task_rule.description, "test")
        self.assertEqual(task_rule.frequency, 1)
        self.assertEqual(task_rule.task_type, TaskTypeEnum.LOOP)
        self.assertEqual(task_rule.frequency_type, FrequencyEnum.ONCE)
        # self.assertEqual(task_rule.time_of_day, now)
        # self.assertEqual(task_rule.start_date, now)

    async def test_create_task_rule_generates_scheduled_task(self):
        now = datetime.utcnow()
        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.ONCE,
            time_of_day=now,
            start_datetime=now,
        )

        created_task = await ScheduledTask.all()
        self.assertEqual(len(created_task), 1)

        self.assertEqual(created_task[0].task_type, TaskTypeEnum.LOOP)
        self.assertEqual(created_task[0].id, 1)

    async def test_delete_task_rule_correctly(self):
        now = datetime.utcnow()
        task_rule = await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.ONCE,
            time_of_day=now,
            start_datetime=now,
        )
        await task_rule.delete()
        self.assertEqual(await TaskRule.all(), [])

    async def test_delete_task_rule_deletes_all_related_scheduled_tasks(self):
        now = datetime.utcnow()
        task_rule = await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=2,
            frequency_type=FrequencyEnum.HOURLY,
            time_of_day=now,
            start_datetime=now,
            end_datetime=now + timedelta(hours=4),
        )
        self.assertEqual(len(await ScheduledTask.all()), 3)

        await task_rule.delete()

        self.assertEqual(await ScheduledTask.all(), [])


class TestCaseCorrectNumberOfScheduledtTaskGeneration(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await start_test_database()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_create_task_rule_generates_scheduled_task_once(self):
        now = datetime.utcnow()
        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.ONCE,
            time_of_day=now,
            start_datetime=now,
        )

        created_task = await ScheduledTask.all()
        self.assertEqual(len(created_task), 1)

        self.assertEqual(created_task[0].task_type, TaskTypeEnum.LOOP)
        self.assertEqual(created_task[0].id, 1)

    async def test_create_task_minutely(self):
        now = datetime.utcnow()
        future = now + timedelta(days=2)

        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=60,
            frequency_type=FrequencyEnum.MINUTELY,
            time_of_day=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_task = await ScheduledTask.all()
        self.assertEqual(len(created_task), 49)

    async def test_create_task_rule_generate_three_scheduled_task(self):
        now = datetime.utcnow()
        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=2,
            frequency_type=FrequencyEnum.HOURLY,
            time_of_day=now,
            start_datetime=now,
            end_datetime=now + timedelta(hours=2),
        )

        created_task = await ScheduledTask.all()
        self.assertEqual(len(created_task), 2)

    async def test_create_task_rule_generate_two_scheduled_task(self):
        now = datetime.utcnow()
        future = now + timedelta(hours=1)
        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=2,
            frequency_type=FrequencyEnum.HOURLY,
            time_of_day=future,
            start_datetime=future,
            end_datetime=now + timedelta(hours=2),
        )

        created_task = await ScheduledTask.all()
        self.assertEqual(len(created_task), 1)

    async def test_create_task_daily(self):
        now = datetime.utcnow()
        future = now + timedelta(days=60)

        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.DAILY,
            time_of_day=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_task = await ScheduledTask.all()
        self.assertEqual(len(created_task), 61)

    async def test_create_task_weekly(self):
        now = datetime.utcnow()
        future = now + timedelta(days=90)

        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.WEEKLY,
            time_of_day=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_task = await ScheduledTask.all()
        self.assertEqual(len(created_task), 13)

    async def test_create_task_monthly(self):
        now = datetime.utcnow()
        future = now + timedelta(days=365)

        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=3,
            frequency_type=FrequencyEnum.MONTHLY,
            time_of_day=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_task = await ScheduledTask.all()
        self.assertEqual(len(created_task), 5)


class TestCaseScheduledtTaskGenerationDateCorrectness(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await start_test_database()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_task_once(self):
        now = datetime.utcnow()
        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.ONCE,
            time_of_day=now,
            start_datetime=now,
        )

        created_task = await ScheduledTask.all()
        self.assertEqual(len(created_task), 1)

        self.assertEqual(created_task[0].task_type, TaskTypeEnum.LOOP)
        self.assertEqual(created_task[0].id, 1)

    async def test_task_hourly(self):
        pass

    async def test_task_daily(self):
        pass

    async def test_task_weekly(self):
        pass

    async def test_task_monthly(self):
        pass

    async def test_monthly_day_of_week(self):
        pass

    async def test_monthly_days_of_week(self):
        pass

    async def test_every_third_day_of_week(self):
        pass

    async def test_every_third_week(self):
        pass

    # TODO: When a specific event occurs (i.e. start, end of another task)
    async def test_on_a_specific_event(self):
        pass
