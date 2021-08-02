from pydantic import BaseModel

from api_server.models import tortoise_models as ttm
from api_server.models.health import basic_health_model
from api_server.models.ros_pydantic import rmf_dispenser_msgs

DispenserHealth = basic_health_model(ttm.DispenserHealth)


class Dispenser(BaseModel):
    guid: str


class DispenserState(rmf_dispenser_msgs.DispenserState):
    @staticmethod
    def from_tortoise(tortoise: ttm.DispenserState) -> "DispenserState":
        return DispenserState(**tortoise.data)

    async def save(self) -> None:
        await ttm.DispenserState.update_or_create({"data": self.dict()}, id_=self.guid)
