from pydantic import BaseModel, Field

from .ros_pydantic import rmf_building_map_msgs, rmf_door_msgs

Door = rmf_building_map_msgs.msg.Door
DoorMode = rmf_door_msgs.msg.DoorMode


DoorState = rmf_door_msgs.msg.DoorState


class DoorRequest(BaseModel):
    mode: int = Field(
        ...,
        description="https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_door_msgs/msg/DoorMode.msg",  # pylint: disable=line-too-long
    )
