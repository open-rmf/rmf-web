from tortoise import fields, models


class ScheduledTask(models.Model):
    id = fields.IntField(pk=True)
    task_type = fields.TextField()
    created_at = fields.DatetimeField(auto_now_add=True)
    task_datetime = fields.DateTimeField()
    rule = fields.ForeignKeyField("models.TaskRule", related_name="rules", null=True)
