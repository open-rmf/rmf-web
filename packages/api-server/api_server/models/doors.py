from pydantic import BaseModel, Field

from . import tortoise_models as ttm
from .ros_pydantic import rmf_building_map_msgs, rmf_door_msgs

Door = rmf_building_map_msgs.Door
DoorMode = rmf_door_msgs.DoorMode
DoorState = rmf_door_msgs.DoorState
DoorHealth = ttm.DoorHealth


class DoorRequest(BaseModel):
    mode: int = Field(
        ...,
        description="https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_door_msgs/msg/DoorMode.msg",  # pylint: disable=line-too-long
    )
