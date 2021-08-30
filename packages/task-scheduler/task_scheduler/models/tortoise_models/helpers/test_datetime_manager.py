import unittest
from datetime import datetime, timedelta

from task_scheduler.models.tortoise_models.helpers.datetime_manager import (
    DatetimeManager,
)


class TestCaseTaskRuleService(unittest.TestCase):
    def test_is_leap_year(self):
        self.assertTrue(DatetimeManager.is_leap_year(2020))

    def test_is_not_leap_year(self):
        self.assertFalse(DatetimeManager.is_leap_year(2021))

    def test_get_leap_year_list(self):
        now = datetime(2020, 8, 2)
        self.assertEqual(DatetimeManager.get_list_of_month_days(now)[2], 29)

    def test_get_normal_year_list(self):
        now = datetime(2021, 8, 2)
        self.assertEqual(DatetimeManager.get_list_of_month_days(now)[2], 28)

    def test_get_month_days_numbers_in_normal_year(self):
        now = datetime(2021, 2, 2)
        days = DatetimeManager.get_month_days_number(now)
        self.assertEqual(days, 28)

    def test_get_month_days_numbers_in_leap_year(self):
        now = datetime(2020, 2, 2)
        days = DatetimeManager.get_month_days_number(now)
        self.assertEqual(days, 29)

    def test_sum_month_days_correctly(self):
        now = datetime(2021, 2, 2)
        days = DatetimeManager.get_sum_of_month_days(2, now)
        self.assertEqual(days, 28 + 31)

    def test_get_delta_days(self):
        now = datetime(2021, 8, 2)
        delta_days = DatetimeManager.get_delta_days(now, 1)
        self.assertEqual((datetime(2021, 9, 6) - now).days, delta_days)

    def test_get_delta_days_2(self):
        now = datetime(2021, 4, 1)
        delta_days = DatetimeManager.get_delta_days(now, 2)
        self.assertEqual((datetime(2021, 6, 3) - now).days, delta_days)

    def test_calculate_next_time_with_date(self):
        """Tests the calculate_next_time method"""
        now = datetime.now()
        delta = timedelta(days=1)
        self.assertEqual(
            DatetimeManager.calculate_next_time(now, delta).date(), (now + delta).date()
        )

    def test_calculate_next_time_with_time(self):
        """Tests the calculate_next_time method with a time"""
        now = datetime.now()
        delta = timedelta(days=1)
        self.assertEqual(
            DatetimeManager.calculate_next_time(now, delta).time(), (now + delta).time()
        )

    def test_calculate_next_time_with_time_and_date(self):
        """Tests the calculate_next_time method with a time and a date"""
        now = datetime.now()
        delta = timedelta(minutes=1)
        self.assertEqual(
            DatetimeManager.calculate_next_time(now, delta).date(),
            (now + delta).date(),
        )

    def test_is_time_between(self):
        """Tests the is_time_between method"""
        now = datetime.now()
        delta = timedelta(minutes=1)
        self.assertTrue(DatetimeManager.is_time_between(now, now + delta, now))
        self.assertTrue(DatetimeManager.is_time_between(now, now + delta, now + delta))
        self.assertFalse(DatetimeManager.is_time_between(now, now + delta, now - delta))
