from pydantic import BaseModel

from . import tortoise_models as ttm
from .health import BasicHealth, HealthStatus
from .ros_pydantic import rmf_dispenser_msgs


class DispenserHealth(BasicHealth):
    @classmethod
    async def from_tortoise_orm(cls, obj: ttm.DispenserHealth) -> "DispenserHealth":
        return DispenserHealth(
            id_=obj.id_,
            health_status=HealthStatus(obj.health_status),
            health_message=obj.health_message,
        )


class Dispenser(BaseModel):
    guid: str


class DispenserState(rmf_dispenser_msgs.DispenserState):
    @staticmethod
    def from_tortoise(tortoise: ttm.DispenserState) -> "DispenserState":
        return DispenserState(**tortoise.data)

    async def save(self) -> None:
        await ttm.DispenserState.update_or_create(
            {"data": self.model_dump()}, id_=self.guid
        )
