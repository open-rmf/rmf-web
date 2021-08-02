from pydantic import BaseModel, Field

from api_server.models import tortoise_models as ttm
from api_server.models.health import basic_health_model
from api_server.models.ros_pydantic import rmf_building_map_msgs, rmf_door_msgs

Door = rmf_building_map_msgs.Door
DoorMode = rmf_door_msgs.DoorMode
DoorHealth = basic_health_model(ttm.DoorHealth)


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
