from typing import Optional, Type

from tortoise.contrib.pydantic.base import PydanticModel

from . import tortoise_models as ttm


class HealthStatus:
    HEALTHY = "Healthy"
    UNHEALTHY = "Unhealthy"
    DEAD = "Dead"


class BaseBasicHealth(PydanticModel):
    id_: str
    health_status: str
    health_message: Optional[str]

    async def save(self) -> None:
        raise NotImplementedError()


def basic_health_model(
    TortoiseModel: Type[ttm.BasicHealthModel],
) -> Type[BaseBasicHealth]:
    """
    Creates a pydantic model from a tortoise basic health model.
    """

    class BasicHealth(TortoiseModel.PydanticModel):
        id_: str
        health_status: str
        health_message: Optional[str]

        @staticmethod
        async def from_tortoise(tortoise: ttm.BasicHealthModel) -> BaseBasicHealth:
            return await TortoiseModel.PydanticModel.from_tortoise_orm(tortoise)

        async def save(self) -> None:
            dic = self.dict()
            del dic["id_"]
            await TortoiseModel.update_or_create(dic, id_=self.id_)

    return BasicHealth
