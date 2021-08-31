# pylint: disable=unused-import
from typing import List, Optional, Type

from apscheduler.jobstores.base import JobLookupError
from tortoise import BaseDBAsyncClient, fields, models
from tortoise.signals import post_delete, post_save

from task_scheduler.scheduler.timer_job import scheduler

from .task import TaskTypeEnum


class ScheduledTask(models.Model):
    id = fields.IntField(pk=True)
    created_at = fields.DatetimeField(auto_now_add=True)
    enabled = fields.BooleanField(default=True)
    rule = fields.ForeignKeyField("models.TaskRule", related_name="rules", null=True)
    task_datetime = fields.DatetimeField()
    task_type: TaskTypeEnum = fields.CharEnumField(TaskTypeEnum)
    # This should contain the task information to run
    task_description = fields.JSONField(null=True)


@post_save(ScheduledTask)
# pylint: disable=unused-argument
async def signal_post_save(
    sender: "Type[ScheduledTask]",
    instance: ScheduledTask,
    created: bool,
    using_db: "Optional[BaseDBAsyncClient]",
    update_fields: List[str],
) -> None:
    def send_task():
        # TODO: send task to ROS
        print("sending task")

    try:
        scheduler.new_job(
            str(instance.id), send_task, (), instance.task_datetime.isoformat()
        )
    except Exception as e:  # pylint: disable=broad-except
        print(e)


@post_delete(ScheduledTask)
# pylint: disable=unused-argument
async def signal_post_delete(
    sender: "Type[ScheduledTask]",
    instance: ScheduledTask,
    using_db: "Optional[BaseDBAsyncClient]",
) -> None:

    try:
        scheduler.delete_job(instance.id)
    except JobLookupError as e:
        print(e)
    except Exception as e:  # pylint: disable=broad-except
        print(e)
