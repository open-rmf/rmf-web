from tortoise import fields, models

from .task import TaskTypeEnum


class ScheduledTask(models.Model):
    id = fields.IntField(pk=True)
    task_type: TaskTypeEnum = fields.CharEnumField(TaskTypeEnum)
    created_at = fields.DatetimeField(auto_now_add=True)
    task_datetime = fields.DatetimeField()
    rule = fields.ForeignKeyField("models.TaskRule", related_name="rules", null=True)
