from abc import ABC, abstractmethod
from typing import Generic, Protocol, TypeVar

from pydantic import BaseModel
from tortoise.contrib.pydantic.creator import pydantic_model_creator
from tortoise.fields import CharField, TextField
from tortoise.models import Model


class BasicHealthModel(Model):
    id_: str = CharField(255, pk=True, source_field="id")  # type: ignore
    health_status: str = CharField(max_length=255, null=True)  # type: ignore
    health_message: str = TextField(null=True)  # type: ignore

    class Meta:
        abstract = True


class DoorHealth(BasicHealthModel):
    pass


class LiftHealth(BasicHealthModel):
    pass


class DispenserHealth(BasicHealthModel):
    pass


class IngestorHealth(BasicHealthModel):
    pass


class RobotHealth(BasicHealthModel):
    pass
