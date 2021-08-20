# conflicts with isort because of local non-relative import
# pylint: disable=wrong-import-order
import unittest
from datetime import datetime, timedelta

from fastapi.testclient import TestClient
from tortoise import Tortoise

from task_scheduler.app import get_app
from task_scheduler.test_utils import start_test_database

from .days_of_week import DaysOfWeek
from .scheduled_task import ScheduledTask
from .task_rule import FrequencyEnum, TaskRule, TaskTypeEnum, WeekDayEnum

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
            first_day_to_apply_rule=now,
            start_datetime=now,
        )

        self.assertEqual(task_rule.description, "test")
        self.assertEqual(task_rule.frequency, 1)
        self.assertEqual(task_rule.task_type, TaskTypeEnum.LOOP)
        self.assertEqual(task_rule.frequency_type, FrequencyEnum.ONCE)

    async def test_create_task_rule_generates_scheduled_task(self):
        now = datetime.utcnow()
        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.ONCE,
            first_day_to_apply_rule=now,
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
            first_day_to_apply_rule=now,
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
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=now + timedelta(hours=4),
        )
        self.assertEqual(len(await ScheduledTask.all()), 3)

        await task_rule.delete()

        self.assertEqual(await ScheduledTask.all(), [])


"""
User picks one date and set a rule based on that.
"""


class TestCaseCorrectNumberOfScheduledtTaskGenerationOneDate(
    unittest.IsolatedAsyncioTestCase
):
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
            first_day_to_apply_rule=now,
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
            first_day_to_apply_rule=now,
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
            first_day_to_apply_rule=now,
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
            first_day_to_apply_rule=future,
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
            first_day_to_apply_rule=now,
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
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_task = await ScheduledTask.all()
        self.assertEqual(len(created_task), 13)

    async def test_create_task_monthly(self):
        now = datetime.utcnow()
        now = now.replace(day=2, month=2)
        future = now + timedelta(days=365)

        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=3,
            frequency_type=FrequencyEnum.MONTHLY,
            first_day_to_apply_rule=now,
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
            first_day_to_apply_rule=now,
            start_datetime=now,
        )

        created_task = await ScheduledTask.all()
        self.assertEqual(len(created_task), 1)
        self.assertEqual(created_task[0].task_datetime.date(), now.date())
        self.assertEqual(created_task[0].task_datetime.time(), now.time())

    async def test_task_hourly(self):
        now = datetime(2021, 8, 2)
        future = now + timedelta(hours=3)

        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=2,
            frequency_type=FrequencyEnum.HOURLY,
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_tasks = await ScheduledTask.all()
        delta = 0
        for task in created_tasks:
            self.assertEqual(task.task_datetime.date(), now.date())
            self.assertEqual(
                task.task_datetime.time(), (now + timedelta(hours=delta)).time()
            )
            delta += 2

    async def test_task_daily(self):
        now = datetime.utcnow()
        future = now + timedelta(days=5)

        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=2,
            frequency_type=FrequencyEnum.DAILY,
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_tasks = await ScheduledTask.all()
        delta = 0
        for task in created_tasks:
            self.assertEqual(task.task_datetime.time(), now.time())
            self.assertEqual(
                task.task_datetime.date(), (now + timedelta(days=delta)).date()
            )
            delta += 2

    async def test_task_weekly(self):
        now = datetime.utcnow()
        future = now + timedelta(days=28)

        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=2,
            frequency_type=FrequencyEnum.WEEKLY,
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_tasks = await ScheduledTask.all()
        delta = 0
        for task in created_tasks:
            self.assertEqual(task.task_datetime.time(), now.time())
            self.assertEqual(
                task.task_datetime.date(), (now + timedelta(days=delta)).date()
            )
            delta += 2 * 7

    async def test_weekly_every_monday(self):
        now = datetime.utcnow()
        now = now.replace(day=2, month=8, year=2021)
        future = now + timedelta(days=30)

        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.WEEKLY,
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_tasks = await ScheduledTask.all()

        for task in created_tasks:
            self.assertEqual(task.task_datetime.time(), now.time())

        def check_correct_date(index, delta):
            self.assertEqual(
                created_tasks[index].task_datetime.date(),
                (now + timedelta(days=delta)).date(),
            )

        check_correct_date(0, 0)
        check_correct_date(1, 7)
        check_correct_date(2, 14)
        check_correct_date(3, 21)

        self.assertEqual(created_tasks[0].task_datetime.weekday(), 0)
        self.assertEqual(created_tasks[1].task_datetime.weekday(), 0)
        self.assertEqual(created_tasks[2].task_datetime.weekday(), 0)
        self.assertEqual(created_tasks[3].task_datetime.weekday(), 0)

    async def test_task_monthly(self):
        now = datetime(2021, 5, 2)
        future = now + timedelta(days=120)
        expect_date = datetime(2021, 7, 4)
        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=2,
            frequency_type=FrequencyEnum.MONTHLY,
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_tasks = await ScheduledTask.all()

        def check_correct_date(index, delta):
            self.assertEqual(
                created_tasks[index].task_datetime.date(),
                (now + timedelta(days=delta)).date(),
            )

        check_correct_date(0, 0)
        check_correct_date(1, (expect_date - now).days)

        for task in created_tasks:
            self.assertEqual(task.task_datetime.time(), now.time())

    async def test_task_monthly_leap_year(self):
        now = datetime(2020, 2, 2)
        future = now + timedelta(days=31)
        expect_date = datetime(2020, 3, 1)

        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.MONTHLY,
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_tasks = await ScheduledTask.all()

        for task in created_tasks:
            self.assertEqual(task.task_datetime.time(), now.time())

        def check_correct_date(index, delta):
            self.assertEqual(
                created_tasks[index].task_datetime.date(),
                (now + timedelta(days=delta)).date(),
            )

        check_correct_date(0, 0)
        check_correct_date(1, (expect_date - now).days)

    async def test_monthly_every_first_monday(self):
        now = datetime.utcnow()
        now = now.replace(day=2, month=8, year=2021, hour=0, minute=0)
        future = now + timedelta(days=60)

        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.MONTHLY,
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=future,
        )

        created_tasks = await ScheduledTask.all()

        def check_correct_date(index, delta):
            self.assertEqual(
                created_tasks[index].task_datetime.date(),
                (now + timedelta(days=delta)).date(),
            )

        check_correct_date(0, 0)
        check_correct_date(1, 35)

    # TODO: When a specific event occurs (i.e. start, end of another task)

    async def test_on_a_specific_event(self):
        pass

    # TODO: Is it holiday?


"""
User picks several week days and set a rule based on that.
"""


class TestCaseTwoWeekdaysScheduledtTaskGenerationDateCorrectness(
    unittest.IsolatedAsyncioTestCase
):
    async def asyncSetUp(self):
        await start_test_database()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_once_monday_and_wednesday(self):
        now = datetime(2021, 8, 2)
        future = now + timedelta(days=14)
        days_of_week = await DaysOfWeek.get_or_create(monday=True, wednesday=True)
        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.ONCE,
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=future,
            days_of_week=days_of_week[0],
        )
        created_tasks = await ScheduledTask.all()
        self.assertEqual(created_tasks[0].task_datetime.weekday(), 0)
        self.assertEqual(created_tasks[1].task_datetime.weekday(), 2)
        self.assertEqual(len(created_tasks), 2)

    async def test_fail_when_daily_hourly_minutely_are_selected(self):
        now = datetime(2021, 8, 2)
        future = now + timedelta(days=14)
        days_of_week = await DaysOfWeek.get_or_create(monday=True, wednesday=True)
        with self.assertRaises(Exception):
            await TaskRule.create(
                description="test",
                task_type=TaskTypeEnum.LOOP,
                frequency=1,
                frequency_type=FrequencyEnum.DAILY,
                first_day_to_apply_rule=now,
                start_datetime=now,
                end_datetime=future,
                days_of_week=days_of_week[0],
            )

            await TaskRule.create(
                description="test",
                task_type=TaskTypeEnum.LOOP,
                frequency=1,
                frequency_type=FrequencyEnum.HOURLY,
                first_day_to_apply_rule=now,
                start_datetime=now,
                end_datetime=future,
                days_of_week=days_of_week[0],
            )

            await TaskRule.create(
                description="test",
                task_type=TaskTypeEnum.LOOP,
                frequency=1,
                frequency_type=FrequencyEnum.MINUTELY,
                first_day_to_apply_rule=now,
                start_datetime=now,
                end_datetime=future,
                days_of_week=days_of_week[0],
            )

    async def test_weekly_monday_and_wednesday(self):
        now = datetime(2021, 8, 2)
        future = now + timedelta(days=13)
        days_of_week = await DaysOfWeek.get_or_create(monday=True, wednesday=True)
        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.WEEKLY,
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=future,
            days_of_week=days_of_week[0],
        )
        created_tasks = await ScheduledTask.all()
        self.assertEqual(
            created_tasks[0].task_datetime.weekday(), WeekDayEnum.MONDAY.value
        )
        self.assertEqual(
            created_tasks[1].task_datetime.weekday(), WeekDayEnum.WEDNESDAY.value
        )
        self.assertEqual(
            created_tasks[2].task_datetime.weekday(), WeekDayEnum.MONDAY.value
        )
        self.assertEqual(
            created_tasks[3].task_datetime.weekday(), WeekDayEnum.WEDNESDAY.value
        )
        self.assertEqual(len(created_tasks), 4)

    async def test_monthly_every_monday_and_wednesday(self):
        now = datetime(2021, 8, 2)
        future = now + timedelta(days=45)
        expect_date = datetime(2020, 3, 1)
        days_of_week = await DaysOfWeek.get_or_create(monday=True, wednesday=True)
        await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.MONTHLY,
            first_day_to_apply_rule=now,
            start_datetime=now,
            end_datetime=future,
            days_of_week=days_of_week[0],
        )
        created_tasks = await ScheduledTask.all()
        self.assertEqual(
            created_tasks[0].task_datetime.weekday(), WeekDayEnum.MONDAY.value
        )
        self.assertEqual(
            created_tasks[1].task_datetime.weekday(), WeekDayEnum.WEDNESDAY.value
        )
        self.assertEqual(
            created_tasks[2].task_datetime.weekday(), WeekDayEnum.MONDAY.value
        )
        self.assertEqual(
            created_tasks[3].task_datetime.weekday(), WeekDayEnum.WEDNESDAY.value
        )
        self.assertEqual(len(created_tasks), 4)


# Set a rule for a weekly monday and start next tuesday.
# Check start date
