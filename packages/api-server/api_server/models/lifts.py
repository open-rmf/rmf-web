from typing import List

from pydantic import BaseModel, Field

from . import tortoise_models as ttm
from .ros_pydantic import rmf_building_map_msgs, rmf_lift_msgs

Lift = rmf_building_map_msgs.Lift
LiftHealth = ttm.LiftHealth


class LiftState(rmf_lift_msgs.LiftState):
    available_modes: List[int]


class LiftRequest(BaseModel):
    request_type: int = Field(
        ...,
        description="https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg",  # pylint: disable=line-too-long
    )
    door_mode: int = Field(
        ...,
        description="https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg",  # pylint: disable=line-too-long
    )
    destination: str
