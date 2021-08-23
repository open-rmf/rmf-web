from tortoise import fields, models

from .task import TaskTypeEnum


class ScheduledTask(models.Model):
    id = fields.IntField(pk=True)
    created_at = fields.DatetimeField(auto_now_add=True)
    enabled = fields.BooleanField(default=True)
    rule = fields.ForeignKeyField("models.TaskRule", related_name="rules", null=True)
    task_datetime = fields.DatetimeField()
    task_type: TaskTypeEnum = fields.CharEnumField(TaskTypeEnum)
