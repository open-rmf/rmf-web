from pydantic import BaseModel

from . import tortoise_models as ttm
from .health import basic_health_model
from .ros_pydantic import rmf_dispenser_msgs

DispenserHealth = basic_health_model(ttm.DispenserHealth)


class Dispenser(BaseModel):
    guid: str


class DispenserState(rmf_dispenser_msgs.DispenserState):
    @staticmethod
    def from_tortoise(tortoise: ttm.DispenserState) -> "DispenserState":
        return DispenserState(**tortoise.data)

    async def save(self) -> None:
        await ttm.DispenserState.update_or_create({"data": self.dict()}, id_=self.guid)
