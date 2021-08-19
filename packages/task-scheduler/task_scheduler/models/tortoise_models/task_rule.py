from datetime import datetime, timedelta
from enum import Enum
from math import ceil
from typing import List, Optional, Type

from tortoise import BaseDBAsyncClient, fields, models
from tortoise.signals import post_delete, post_save, pre_save

from .days_of_week import DaysOfWeek
from .helpers.task_rule import calculate_next_time, is_time_between
from .scheduled_task import ScheduledTask
from .task import TaskTypeEnum


class FrequencyEnum(str, Enum):
    MONTHLY = "Monthly"
    WEEKLY = "Weekly"
    DAILY = "Daily"
    HOURLY = "Hourly"
    MINUTELY = "Minutely"
    ONCE = "Once"
    CUSTOM = "Custom"


MONTH_DAYS = [0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
MONTH_DAYS_LEAP = [0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]


class TaskRuleService:
    # Check if a day is a leap year
    def is_leap_year(self, year: int) -> int:
        return (year % 4 == 0 and year % 100 != 0) or year % 400 == 0

    def get_sum_of_month_days(self, frequency: int, datetime: datetime) -> int:
        current_month: int = datetime.month
        total_days = 0
        month_list = self.get_list_of_month_days(datetime)
        if frequency == 1:
            return month_list[datetime.month]
        for month in range(current_month, current_month + frequency):
            total_days += month_list[month]
        return total_days

    # Check the number of days of a month including leap years
    def get_month_days_number(self, datetime: datetime):
        month_list = self.get_list_of_month_days(datetime)
        return month_list[datetime.month]

    def get_list_of_month_days(self, datetime: datetime) -> List[int]:
        year = datetime.year
        if self.is_leap_year(year):
            return MONTH_DAYS_LEAP
        return MONTH_DAYS

    def week_of_month(self, dt):

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

    def is_next_month(
        self, current: datetime, new_date: datetime, frequency: int
    ) -> bool:
        current_month: int = current.month
        new_month: int = new_date.month
        return (current_month + frequency) == new_month

    def is_same_week(self, current: datetime, new_date: datetime) -> bool:
        current_week = self.week_of_month(current)
        new_week = self.week_of_month(new_date)
        return current_week == new_week

    def get_delta_days(self, current: datetime, frequency: int) -> int:
        # Months can have 4 to 6 week but not less than 4
        # 4 weeks = 28 days
        basic_days = 28 * frequency
        new_date = current + timedelta(days=basic_days)

        def is_next_month() -> bool:
            return self.is_next_month(current, new_date, frequency)

        def is_same_week() -> bool:
            return self.is_same_week(current, new_date)

        while not is_next_month() and not is_same_week():
            new_date = new_date + timedelta(days=7)
        # get the days of difference between current and new date
        days = (new_date - current).days
        return days

    def get_timedelta(self, delta_type, frequency, datetime=None):
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

            return timedelta(days=self.get_delta_days(datetime, frequency))

        elif delta_type == FrequencyEnum.CUSTOM:
            if datetime is None:
                raise ValueError("Month can't be None")

            return timedelta(days=self.get_delta_days(datetime, frequency))
        else:
            return None


class TaskRule(models.Model):
    id = models.IntField(pk=True)
    description = fields.CharField(unique=True, max_length=150)
    task_type: TaskTypeEnum = fields.CharEnumField(TaskTypeEnum)
    frequency = fields.IntField()
    frequency_type: FrequencyEnum = fields.CharEnumField(
        FrequencyEnum, default=FrequencyEnum.MINUTELY
    )
    first_day_to_apply_rule = fields.DatetimeField(null=True)
    created_at = fields.DatetimeField(auto_now_add=True)
    start_datetime = fields.DatetimeField()
    end_datetime = fields.DatetimeField(null=True)
    # args = fields.JSONField()
    service = TaskRuleService()
    days_of_week = fields.ForeignKeyField(
        "models.DaysOfWeek", related_name="task_rule", null=True
    )


class SimpleScheduleManager:
    @staticmethod
    async def handleScheduleWithOneDate(instance):
        last_task_datetime = instance.start_datetime

        if instance.frequency_type == FrequencyEnum.ONCE:
            await ScheduledTask.create(
                task_type=instance.task_type,
                # Need to modify this
                task_datetime=instance.first_day_to_apply_rule,
                rule=instance,
            )
            return

        time_delta_content = TaskRule.service.get_timedelta(
            instance.frequency_type, instance.frequency, instance.start_datetime
        )

        while is_time_between(
            instance.start_datetime, instance.end_datetime, last_task_datetime
        ):
            await ScheduledTask.create(
                task_type=instance.task_type,
                # Need to modify this
                task_datetime=last_task_datetime,
                rule=instance,
            )
            last_task_datetime = calculate_next_time(
                last_task_datetime, time_delta_content
            )


class MultipleDaysScheduleManager:
    @staticmethod
    async def handleScheduleWithMultipleDays(instance, week_id):
        # We pick first_day_to_apply_rule because the task can start with
        # earlier but not the execution
        last_task_datetime = instance.first_day_to_apply_rule
        # [0,6]
        active_weekdays = await DaysOfWeek.service.get_active_days_of_week(week_id)

        if (
            instance.frequency_type == FrequencyEnum.DAILY
            or instance.frequency_type == FrequencyEnum.HOURLY
            or instance.frequency_type == FrequencyEnum.MINUTELY
        ):
            raise Exception(
                "You cannot set the rule DAILY|HOURLY|MINUTELY and pick weekdays"
            )

        if instance.frequency_type == FrequencyEnum.ONCE:

            week_day_tasks = MultipleDaysScheduleManager._get_initialized_datetime_list(
                active_weekdays, instance.first_day_to_apply_rule
            )
            for weekday in week_day_tasks:
                if weekday is None:
                    continue

                await ScheduledTask.create(
                    task_type=instance.task_type,
                    task_datetime=weekday,
                    rule=instance,
                )
            return

        # Get next week, month
        # time_delta_content = TaskRule.service.get_timedelta(
        #     instance.frequency_type, instance.frequency, instance.start_datetime
        # )

        week_day_tasks = MultipleDaysScheduleManager._get_initialized_datetime_list(
            active_weekdays, instance.first_day_to_apply_rule
        )

        while is_time_between(
            instance.start_datetime,
            instance.end_datetime,
            week_day_tasks[active_weekdays[0]],
        ):
            # for active_weekday in active_weekdays:
            for index, weekday_datetime in enumerate(week_day_tasks):
                if weekday_datetime is None:
                    continue

                if instance.end_datetime < weekday_datetime:
                    break

                await ScheduledTask.create(
                    task_type=instance.task_type,
                    task_datetime=weekday_datetime,
                    rule=instance,
                )
                # Add the frecuency again
                week_day_tasks[index] = calculate_next_time(
                    weekday_datetime, timedelta(days=7)
                )

    # Calculate date distance between weekdays

    @staticmethod
    def get_weekday_distance(weekday_1: int, weekday_2: int) -> int:
        return abs(weekday_1 - weekday_2)

    # Get first day of next week
    @staticmethod
    def get_monday_of_next_week(current_day: datetime) -> datetime:
        return current_day + timedelta(days=7 - current_day.weekday())

    @staticmethod
    def _get_initialized_datetime_list(
        active_weekdays: List[int], last_task_datetime
    ) -> List[datetime]:
        week_day_tasks: List[datetime] = [None, None, None, None, None, None, None]
        count = 0
        # Initialize week with first values
        for weekday in active_weekdays:
            if count == 0:
                week_day_tasks[weekday] = last_task_datetime

            weekday_distance = MultipleDaysScheduleManager.get_weekday_distance(
                weekday, last_task_datetime.weekday()
            )

            week_day_tasks[weekday] = calculate_next_time(
                last_task_datetime, timedelta(days=weekday_distance)
            )
        return week_day_tasks


@post_save(TaskRule)
async def signal_post_save(
    sender: "Type[TaskRule]",
    instance: TaskRule,
    created: bool,
    using_db: "Optional[BaseDBAsyncClient]",
    update_fields: List[str],
) -> None:

    task_rule_instance = await TaskRule.get(id=instance.id).prefetch_related(
        "days_of_week"
    )

    if task_rule_instance.days_of_week is None:
        instance.first_day_to_apply_rule = instance.start_datetime
        await SimpleScheduleManager.handleScheduleWithOneDate(instance)
    else:
        instance.first_day_to_apply_rule = (
            await DaysOfWeek.service.get_first_active_day(
                id=task_rule_instance.days_of_week.id,
                current_datetime=task_rule_instance.start_datetime,
            )
        )
        await MultipleDaysScheduleManager.handleScheduleWithMultipleDays(
            instance, task_rule_instance.days_of_week.id
        )


@post_delete(TaskRule)
async def signal_post_delete(
    sender: "Type[TaskRule]",
    instance: TaskRule,
    using_db: "Optional[BaseDBAsyncClient]",
) -> None:
    tasks = await ScheduledTask.filter(rule=instance).all()
    for task in tasks:
        await task.delete()
