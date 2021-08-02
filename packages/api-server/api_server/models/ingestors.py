from pydantic import BaseModel

from . import tortoise_models as ttm
from .health import basic_health_model
from .ros_pydantic import rmf_ingestor_msgs

IngestorHealth = basic_health_model(ttm.IngestorHealth)


class Ingestor(BaseModel):
    guid: str


class IngestorState(rmf_ingestor_msgs.IngestorState):
    @staticmethod
    def from_tortoise(tortoise: ttm.IngestorState) -> "IngestorState":
        return IngestorState(**tortoise.data)

    async def save(self) -> None:
        await ttm.IngestorState.update_or_create({"data": self.dict()}, id_=self.guid)
