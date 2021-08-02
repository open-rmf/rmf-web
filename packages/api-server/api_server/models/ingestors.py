from pydantic import BaseModel

from api_server.models import tortoise_models as ttm
from api_server.models.health import basic_health_model
from api_server.models.ros_pydantic import rmf_ingestor_msgs

IngestorHealth = basic_health_model(ttm.IngestorHealth)


class Ingestor(BaseModel):
    guid: str


class IngestorState(rmf_ingestor_msgs.IngestorState):
    @staticmethod
    def from_tortoise(tortoise: ttm.IngestorState) -> "IngestorState":
        return IngestorState(**tortoise.data)

    async def save(self) -> None:
        await ttm.IngestorState.update_or_create({"data": self.dict()}, id_=self.guid)
