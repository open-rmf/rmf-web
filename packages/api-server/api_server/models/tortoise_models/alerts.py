from datetime import datetime
from enum import Enum
from typing import Optional

from tortoise.contrib.pydantic.creator import pydantic_model_creator
from tortoise.fields import CharEnumField, CharField, DatetimeField
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
    category = CharEnumField(Category)
    created_on = DatetimeField()
    acknowledged_by = CharField(255, null=True)
    acknowledged_on: Optional[datetime] = DatetimeField(null=True)


AlertPydantic = pydantic_model_creator(Alert)
