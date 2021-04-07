from tortoise import fields, models
from tortoise.contrib.pydantic import pydantic_model_creator


class RawLog(models.Model):
    id = fields.IntField(pk=True)
    # JSONField https://tortoise-orm.readthedocs.io/en/latest/fields.html#tortoise.fields.data.IntField.field_type
    payload = fields.JSONField()
    created = fields.DatetimeField(auto_now_add=True)

    # def __str__(self):
    #     return self.payload


RawLog_Pydantic = pydantic_model_creator(RawLog, name="RawLog")
