from tortoise import Model, fields
from tortoise.contrib.pydantic.base import PydanticModel
from tortoise.contrib.pydantic.creator import pydantic_model_creator

from api_server.models.tortoise_models.health_status_mixin import HealthStatusMixin


class BasicHealthModel(Model, HealthStatusMixin):
    PydanticModel: PydanticModel

    id_ = fields.CharField(255, pk=True, source_field="id")

    async def to_pydantic(self):
        return await self.PydanticModel.from_tortoise_orm(self)


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
