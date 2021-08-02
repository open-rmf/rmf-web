from tortoise import fields, models


class DaysOfWeek(models.Model):
    id = fields.IntField(pk=True)
    monday = fields.BooleanField(default=False)
    tuesday = fields.BooleanField(default=False)
    wednesday = fields.BooleanField(default=False)
    thursday = fields.BooleanField(default=False)
    friday = fields.BooleanField(default=False)
    saturday = fields.BooleanField(default=False)
    sunday = fields.BooleanField(default=False)
    rule = fields.ForeignKeyField(
        "models.TaskRule", related_name="days_of_week", null=True
    )
