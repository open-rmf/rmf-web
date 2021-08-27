import unittest
from datetime import datetime

from fastapi.testclient import TestClient
from tortoise import Tortoise

from task_scheduler.app import get_app
from task_scheduler.test_utils import start_test_database

from .days_of_week import DaysOfWeek

app = get_app()


class TestCaseDaysOfWeekService(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await start_test_database()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_get_list_of_week_days(self):
        await DaysOfWeek.create(monday=True, sunday=True)
        # pylint: disable=W0212
        weekdays = await DaysOfWeek.service._get_days_of_week(week_id=1)
        self.assertEqual(weekdays, [True, False, False, False, False, False, True])

    async def test_get_first_active_day_on_week(self):
        now = datetime.utcnow()
        now = now.replace(year=2021, month=8, day=2)
        await DaysOfWeek.create(friday=True, saturday=True)
        active_date = await DaysOfWeek.service.get_first_active_day(
            week_id=1, current_datetime=now
        )
        new_date = now.replace(year=2021, month=8, day=6)
        self.assertEqual(active_date.date(), new_date.date())

    async def test_get_first_active_day_on_next_week(self):
        now = datetime.utcnow()
        now = now.replace(year=2021, month=8, day=3)
        await DaysOfWeek.create(monday=True)
        active_date = await DaysOfWeek.service.get_first_active_day(
            week_id=1, current_datetime=now
        )
        new_date = now.replace(year=2021, month=8, day=9)
        self.assertEqual(active_date.date(), new_date.date())
