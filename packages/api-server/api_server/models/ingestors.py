from pydantic import BaseModel
from tortoise.contrib.pydantic.base import PydanticModel
from tortoise.contrib.pydantic.creator import pydantic_model_creator

from . import tortoise_models as ttm
from .health import BaseBasicHealthModel
from .ros_pydantic import rmf_ingestor_msgs


class IngestorHealth(pydantic_model_creator(ttm.IngestorHealth), BaseBasicHealthModel):
    pass


class Ingestor(BaseModel):
    guid: str


class IngestorState(rmf_ingestor_msgs.IngestorState):
    @staticmethod
    def from_tortoise(tortoise: ttm.IngestorState) -> "IngestorState":
        return IngestorState(**tortoise.data)

    async def save(self) -> None:
        await ttm.IngestorState.update_or_create({"data": self.dict()}, id_=self.guid)
