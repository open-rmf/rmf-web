from tortoise import Model, fields
from tortoise.contrib.pydantic.base import PydanticModel
from tortoise.contrib.pydantic.creator import pydantic_model_creator

from .health_status_mixin import HealthStatusMixin


class BasicHealthModel(Model, HealthStatusMixin):
    PydanticModel: PydanticModel

    id_ = fields.CharField(255, pk=True, source_field="id")


BasicHealthModel.PydanticModel = pydantic_model_creator(BasicHealthModel)


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
