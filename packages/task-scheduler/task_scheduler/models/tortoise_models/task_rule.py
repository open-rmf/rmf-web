from datetime import datetime, timedelta
from enum import Enum
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

    def get_timedelta(self, delta_type, frequency, datetime=None):
        if delta_type == FrequencyEnum.MINUTELY:
            return timedelta(minutes=frequency)
        elif delta_type == FrequencyEnum.HOURLY:
            return timedelta(hours=frequency)
        elif delta_type == FrequencyEnum.DAILY:
            return timedelta(days=frequency)
        elif delta_type == FrequencyEnum.WEEKLY:
            return timedelta(days=frequency * 7)
        elif delta_type == FrequencyEnum.MONTHLY:
            if datetime is None:
                raise ValueError("Month can't be None")

            return timedelta(days=self.get_sum_of_month_days(frequency, datetime))

        elif delta_type == FrequencyEnum.CUSTOM:
            if datetime is None:
                raise ValueError("Month can't be None")

            return timedelta(days=self.get_sum_of_month_days(frequency, datetime))
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


# @pre_save(TaskRule)
# async def signal_post_save(
#     sender: "Type[TaskRule]",
#     instance: TaskRule,
#     created: bool,
#     using_db: "Optional[BaseDBAsyncClient]",
#     update_fields: List[str],
# ) -> None:
#     last_task_datetime = instance.start_datetime
#     if instance.frequency_type == FrequencyEnum.ONCE:
#         await ScheduledTask.create(
#             task_type=instance.task_type,
#             # Need to modify this
#             task_datetime=instance.first_day_to_apply_rule,
#             rule=instance,
#         )
#         return

#     time_delta_content = TaskRule.service.get_timedelta(
#         instance.frequency_type, instance.frequency, instance.start_datetime
#     )

#     while is_time_between(
#         instance.start_datetime, instance.end_datetime, last_task_datetime
#     ):
#         await ScheduledTask.create(
#             task_type=instance.task_type,
#             # Need to modify this
#             task_datetime=last_task_datetime,
#             rule=instance,
#         )
#         last_task_datetime = calculate_next_time(
#             last_task_datetime, time_delta_content)


@post_save(TaskRule)
async def signal_post_save(
    sender: "Type[TaskRule]",
    instance: TaskRule,
    created: bool,
    using_db: "Optional[BaseDBAsyncClient]",
    update_fields: List[str],
) -> None:

    # print(instance.days_of_week['id'])
    print(instance.id)
    hi = await TaskRule.get(id=instance.id).prefetch_related("days_of_week")
    print("hehe", hi.days_of_week)
    if instance.days_of_week is None:
        instance.first_day_to_apply_rule = instance.start_datetime
    else:
        pass
        # instance.first_day_to_apply_rule = await DaysOfWeek.service.get_first_active_day(
        #     id=instance.days_of_week.id,
        #     current_datetime=instance.start_datetime
        # )

    # instance.save()

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
        last_task_datetime = calculate_next_time(last_task_datetime, time_delta_content)


@post_delete(TaskRule)
async def signal_post_delete(
    sender: "Type[TaskRule]",
    instance: TaskRule,
    using_db: "Optional[BaseDBAsyncClient]",
) -> None:
    tasks = await ScheduledTask.filter(rule=instance).all()
    for task in tasks:
        await task.delete()


# id SERIAL UNIQUE,                      -- unique identifier for the job
# name varchar(64) NOT NULL,             -- human readable name for the job
# description text,                      -- details about the job
# schedule varchar(64) NOT NULL,         -- valid CRON expression for the job schedule
# handler varchar(64) NOT NULL,          -- string representing handler for the job
# args text NOT NULL,                    -- arguments for the job handler
# enabled boolean NOT NULL DEFAULT TRUE, -- whether the job should be run
# created_at timestamp NOT NULL,         -- when was the job created
# updated_at timestamp NOT NULL,         -- when was the job updated
# start_date timestamp,                  -- job should not run until this time
# end_date timestamp,                    -- job should not run after this time
# last_triggered_at timestamp,           -- when was the job last triggered
# meta json                              -- additional metadata for the job
