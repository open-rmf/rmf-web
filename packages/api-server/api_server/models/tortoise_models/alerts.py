from datetime import datetime
from enum import Enum
from typing import Optional

from tortoise.contrib.pydantic.creator import pydantic_model_creator
from tortoise.fields import CharEnumField, CharField, DatetimeField, FloatField
from tortoise.models import Model
from tortoise.validators import MaxValueValidator, MinValueValidator


class AlertDetails(Model):
    class Category(str, Enum):
        Default = "default"
        Success = "success"
        Warning = "warning"
        Error = "error"

    id = CharField(255, pk=True)
    title = CharField(255)
    category = CharEnumField(Category)
    progress = FloatField(
        null=True, validators=[MaxValueValidator(1.0), MinValueValidator(0.0)]
    )
    description = CharField(255, null=True)


class Alert(Model):
    """
    General alert that can be triggered by events.
    """

    id = CharField(255, pk=True)
    created_on = DatetimeField()
    detail: AlertDetails
    acknowledged_by = CharField(255, null=True)
    acknowledged_on: Optional[datetime] = DatetimeField(null=True)


AlertDetailsPydantic = pydantic_model_creator(AlertDetails)
AlertPydantic = pydantic_model_creator(Alert)
