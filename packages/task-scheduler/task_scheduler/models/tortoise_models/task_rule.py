from typing import List, Optional, Type

from tortoise import BaseDBAsyncClient, fields, models
from tortoise.signals import post_delete, post_save, pre_save

from .helpers.task_rule_definition import FrequencyEnum
from .helpers.task_rule_manager import SchedulerFactory
from .scheduled_task import ScheduledTask
from .task import TaskTypeEnum


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
    # service = TaskRuleService()
    days_of_week = fields.ForeignKeyField(
        "models.DaysOfWeek", related_name="task_rule", null=True
    )


@pre_save(TaskRule)
async def signal_pre_save(
    sender: "Type[TaskRule]",
    instance: TaskRule,
    using_db: "Optional[BaseDBAsyncClient]",
    update_fields: List[str],
) -> None:

    if instance.frequency_type != FrequencyEnum.ONCE and instance.end_datetime is None:
        raise Exception("You should set a end_datetime")


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

    schedule_manager = SchedulerFactory.get_schedule_manager(task_rule_instance)

    await schedule_manager.schedule_tasks()


@post_delete(TaskRule)
async def signal_post_delete(
    sender: "Type[TaskRule]",
    instance: TaskRule,
    using_db: "Optional[BaseDBAsyncClient]",
) -> None:
    tasks = await ScheduledTask.filter(rule=instance).all()
    for task in tasks:
        await task.delete()
