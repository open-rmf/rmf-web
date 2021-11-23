from abc import ABC, abstractmethod
from typing import Generic, Optional, Type, TypeVar, cast

from tortoise.contrib.pydantic.base import PydanticModel
from tortoise.contrib.pydantic.creator import pydantic_model_creator

from . import tortoise_models as ttm


class HealthStatus:
    HEALTHY = "Healthy"
    UNHEALTHY = "Unhealthy"
    DEAD = "Dead"


HealthModelT = TypeVar("HealthModelT", bound=ttm.BasicHealthModel)


class BaseBasicHealth(Generic[HealthModelT], ABC, PydanticModel):
    id_: str
    health_status: str
    health_message: Optional[str]

    @staticmethod
    @abstractmethod
    async def from_tortoise(_tortoise: HealthModelT) -> "BaseBasicHealth":
        pass

    @abstractmethod
    async def save(self) -> None:
        pass


def basic_health_model(
    TortoiseModel: Type[HealthModelT],
) -> Type[BaseBasicHealth[HealthModelT]]:
    """
    Creates a pydantic model from a tortoise basic health model.
    """

    class _BasicHealth(pydantic_model_creator(TortoiseModel)):
        id_: str
        health_status: str
        health_message: Optional[str]

        @classmethod
        async def from_tortoise(cls, tortoise: ttm.BasicHealthModel):
            return await cls.from_tortoise_orm(tortoise)

        async def save(self):
            dic = self.dict()
            del dic["id_"]
            await TortoiseModel.update_or_create(dic, id_=self.id_)

    _BasicHealth.__name__ = TortoiseModel.__name__

    return cast(Type[BaseBasicHealth[HealthModelT]], _BasicHealth)
