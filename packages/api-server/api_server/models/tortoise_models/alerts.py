from enum import Enum

from tortoise.contrib.pydantic.creator import pydantic_model_creator
from tortoise.fields import BigIntField, CharEnumField, CharField
from tortoise.models import Model


class Alert(Model):
    """
    General alert that can be triggered by events.
    """

    class Category(str, Enum):
        Default = "default"
        Task = "task"
        Fleet = "fleet"
        Robot = "robot"

    id = CharField(255, pk=True)
    original_id = CharField(255, index=True)
    category = CharEnumField(Category, index=True)
    unix_millis_created_time = BigIntField(null=False, index=True)
    acknowledged_by = CharField(255, null=True, index=True)
    unix_millis_acknowledged_time = BigIntField(null=True, index=True)


AlertPydantic = pydantic_model_creator(Alert)
