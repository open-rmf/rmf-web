from datetime import datetime, timedelta
from math import ceil
from typing import List

from .task_rule_definition import MONTH_DAYS, MONTH_DAYS_LEAP, FrequencyEnum


class DatetimeManager:
    @staticmethod
    def is_time_between(start_time, end_time, current_time) -> bool:
        return start_time <= current_time <= end_time

    @staticmethod
    def calculate_next_time(current_time, delta) -> datetime:
        return current_time + delta

    @staticmethod
    def is_leap_year(year: int) -> int:
        return (year % 4 == 0 and year % 100 != 0) or year % 400 == 0

    @staticmethod
    def get_sum_of_month_days(frequency: int, datetime: datetime) -> int:
        current_month: int = datetime.month
        total_days = 0
        month_list = DatetimeManager.get_list_of_month_days(datetime)
        if frequency == 1:
            return month_list[datetime.month]
        for month in range(current_month, current_month + frequency):
            total_days += month_list[month]
        return total_days

    @staticmethod
    # Check the number of days of a month including leap years
    def get_month_days_number(datetime: datetime):
        month_list = DatetimeManager.get_list_of_month_days(datetime)
        return month_list[datetime.month]

    @staticmethod
    def get_list_of_month_days(datetime: datetime) -> List[int]:
        year = datetime.year
        if DatetimeManager.is_leap_year(year):
            return MONTH_DAYS_LEAP
        return MONTH_DAYS

    @staticmethod
    def week_of_month(dt):
        first_day = dt.replace(day=1)
        dom = dt.day
        if first_day.weekday() == 6:
            adjusted_dom = dom
        else:
            adjusted_dom = dom + first_day.weekday()
        if adjusted_dom % 7 == 0 and first_day.weekday() != 6:
            value = adjusted_dom / 7.0 + 1
        elif first_day.weekday() == 6 and adjusted_dom % 7 == 0 and adjusted_dom == 7:
            value = 1
        else:
            value = int(ceil(adjusted_dom / 7.0))

        return int(value)

    @staticmethod
    def is_next_month(current: datetime, new_date: datetime, frequency: int) -> bool:
        current_month: int = current.month
        new_month: int = new_date.month
        return (current_month + frequency) == new_month

    @staticmethod
    def is_same_week(current: datetime, new_date: datetime) -> bool:
        current_week = DatetimeManager.week_of_month(current)
        new_week = DatetimeManager.week_of_month(new_date)
        return current_week == new_week

    @staticmethod
    def get_delta_days(current: datetime, frequency: int) -> int:
        # Months can have 4 to 6 week but not less than 4
        # 4 weeks = 28 days
        basic_days = 28 * frequency
        new_date = current + timedelta(days=basic_days)

        def is_next_month() -> bool:
            return DatetimeManager.is_next_month(current, new_date, frequency)

        def is_same_week() -> bool:
            return DatetimeManager.is_same_week(current, new_date)

        while not is_next_month() and not is_same_week():
            new_date = new_date + timedelta(days=7)
        # get the days of difference between current and new date
        days = (new_date - current).days
        return days

    @staticmethod
    def get_timedelta(delta_type, frequency, datetime=None):
        if delta_type == FrequencyEnum.MINUTELY:
            return timedelta(minutes=frequency)
        elif delta_type == FrequencyEnum.HOURLY:
            return timedelta(hours=frequency)
        elif delta_type == FrequencyEnum.DAILY:
            return timedelta(days=frequency)
        elif delta_type == FrequencyEnum.WEEKLY:
            return timedelta(days=frequency * 7)
        # Monthly on weekdays
        elif delta_type == FrequencyEnum.MONTHLY:
            if datetime is None:
                raise ValueError("Month can't be None")

            return timedelta(days=DatetimeManager.get_delta_days(datetime, frequency))

        elif delta_type == FrequencyEnum.CUSTOM:
            if datetime is None:
                raise ValueError("Month can't be None")

            return timedelta(days=DatetimeManager.get_delta_days(datetime, frequency))
        else:
            return None
