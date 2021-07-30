from enum import Enum
from typing import List, Optional, Type

from tortoise import BaseDBAsyncClient, fields, models
from tortoise.signals import post_delete, post_save, pre_delete, pre_save

# type (minutes, hours, Daily, monthly, weekly, fixed, yearly) - Enum


class FrequencyEnum(str, Enum):
    YEARLY = "Yearly"
    MONTHLY = "Monthly"
    WEEKLY = "Weekly"
    DAILY = "Daily"
    HOURLY = "Hourly"
    MINUTELY = "Minutely"
    ONCE = "Once"


class TaskTypeEnum(str, Enum):
    CLEAN = "clean"
    LOOP = "loop"
    DELIVERY = "delivery"


class TaskRule(models.Model):
    id = models.IntField(pk=True)
    description = fields.TextField()
    task_type: TaskTypeEnum = fields.CharEnumField(TaskTypeEnum)
    frequency = fields.IntField()
    frequency_type: FrequencyEnum = fields.CharEnumField(
        FrequencyEnum, default=FrequencyEnum.MINUTELY
    )
    time_of_day = fields.DatetimeField()
    created_at = fields.DatetimeField(auto_now_add=True)
    start_date = fields.DatetimeField()
    end_date = fields.DatetimeField(null=True)
    # args = fields.JSONField()


@post_save(TaskRule)
async def signal_post_save(
    sender: "Type[TaskRule]",
    instance: TaskRule,
    created: bool,
    using_db: "Optional[BaseDBAsyncClient]",
    update_fields: List[str],
) -> None:
    print(instance.description)
    print(sender, instance, using_db, created, update_fields)


@post_delete(TaskRule)
async def signal_post_delete(
    sender: "Type[TaskRule]",
    instance: TaskRule,
    using_db: "Optional[BaseDBAsyncClient]",
) -> None:
    print(sender, instance, using_db)


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
