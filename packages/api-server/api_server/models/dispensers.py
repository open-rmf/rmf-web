from pydantic import BaseModel
from tortoise.contrib.pydantic.creator import pydantic_model_creator

from . import tortoise_models as ttm
from .health import BaseBasicHealthModel
from .ros_pydantic import rmf_dispenser_msgs


class DispenserHealth(
    pydantic_model_creator(ttm.DispenserHealth), BaseBasicHealthModel
):
    pass


class Dispenser(BaseModel):
    guid: str


class DispenserState(rmf_dispenser_msgs.DispenserState):
    @staticmethod
    def from_tortoise(tortoise: ttm.DispenserState) -> "DispenserState":
        return DispenserState(**tortoise.data)

    async def save(self) -> None:
        await ttm.DispenserState.update_or_create({"data": self.dict()}, id_=self.guid)
