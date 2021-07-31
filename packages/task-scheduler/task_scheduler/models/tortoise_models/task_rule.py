from datetime import datetime, timedelta
from enum import Enum
from typing import List, Optional, Type

from tortoise import BaseDBAsyncClient, fields, models
from tortoise.signals import post_delete, post_save, pre_delete, pre_save

from .helpers.task_rule import calculate_next_time, is_time_between
from .scheduled_task import ScheduledTask
from .task import TaskTypeEnum

# type (minutes, hours, Daily, monthly, weekly, fixed, yearly) - Enum


class FrequencyEnum(str, Enum):
    YEARLY = "Yearly"
    MONTHLY = "Monthly"
    WEEKLY = "Weekly"
    DAILY = "Daily"
    HOURLY = "Hourly"
    MINUTELY = "Minutely"
    ONCE = "Once"


class TaskRuleService:
    def get_timedelta(delta_type, frequency):
        if delta_type == FrequencyEnum.MINUTELY:
            return timedelta(minutes=frequency)
        elif delta_type == FrequencyEnum.HOURLY:
            return timedelta(hours=frequency)
        elif delta_type == FrequencyEnum.DAILY:
            return timedelta(days=frequency)
        elif delta_type == FrequencyEnum.WEEKLY:
            return timedelta(days=frequency * 7)
        elif delta_type == FrequencyEnum.MONTHLY:
            return timedelta(days=frequency * 30)
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
    time_of_day = fields.DatetimeField()
    created_at = fields.DatetimeField(auto_now_add=True)
    start_datetime = fields.DatetimeField()
    end_datetime = fields.DatetimeField(null=True)
    # args = fields.JSONField()
    service = TaskRuleService


@post_save(TaskRule)
async def signal_post_save(
    sender: "Type[TaskRule]",
    instance: TaskRule,
    created: bool,
    using_db: "Optional[BaseDBAsyncClient]",
    update_fields: List[str],
) -> None:
    last_task_datetime = instance.start_datetime
    if instance.frequency_type == FrequencyEnum.ONCE:
        await ScheduledTask.create(
            task_type=instance.task_type,
            # Need to modify this
            task_datetime=instance.time_of_day,
            rule=instance,
        )
        return

    time_delta_content = TaskRule.service.get_timedelta(
        instance.frequency_type, instance.frequency
    )
    print(
        is_time_between(
            instance.start_datetime, instance.end_datetime, last_task_datetime
        )
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
        print(last_task_datetime)


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
