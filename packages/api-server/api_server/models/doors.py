from pydantic import BaseModel, Field
from tortoise.contrib.pydantic.base import PydanticModel
from tortoise.contrib.pydantic.creator import pydantic_model_creator
from tortoise.models import Model

from . import tortoise_models as ttm
from .health import BaseBasicHealthModel
from .ros_pydantic import rmf_building_map_msgs, rmf_door_msgs

Door = rmf_building_map_msgs.Door
DoorMode = rmf_door_msgs.DoorMode


class DoorHealth(pydantic_model_creator(ttm.LiftHealth), BaseBasicHealthModel):
    pass


class DoorState(rmf_door_msgs.DoorState):
    @staticmethod
    def from_tortoise(tortoise: ttm.DoorState) -> "DoorState":
        return DoorState(**tortoise.data)

    async def save(self) -> None:
        await ttm.DoorState.update_or_create({"data": self.dict()}, id_=self.door_name)


class DoorRequest(BaseModel):
    mode: int = Field(
        ...,
        description="https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_door_msgs/msg/DoorMode.msg",  # pylint: disable=line-too-long
    )
