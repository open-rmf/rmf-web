from pydantic import BaseModel

from . import tortoise_models as ttm
from .health import BasicHealth, HealthStatus
from .ros_pydantic import rmf_ingestor_msgs


class IngestorHealth(BasicHealth):
    @classmethod
    async def from_tortoise_orm(cls, obj: ttm.IngestorHealth) -> "IngestorHealth":
        return IngestorHealth(
            id_=obj.id_,
            health_status=HealthStatus(obj.health_status),
            health_message=obj.health_message,
        )


class Ingestor(BaseModel):
    guid: str


class IngestorState(rmf_ingestor_msgs.IngestorState):
    @staticmethod
    def from_tortoise(tortoise: ttm.IngestorState) -> "IngestorState":
        return IngestorState(**tortoise.data)

    async def save(self) -> None:
        await ttm.IngestorState.update_or_create(
            {"data": self.model_dump()}, id_=self.guid
        )
