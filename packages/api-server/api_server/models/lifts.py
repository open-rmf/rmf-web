from typing import Annotated

from pydantic import BaseModel, Field

from . import tortoise_models as ttm
from .ros_pydantic import builtin_interfaces, rmf_building_map_msgs

Lift = rmf_building_map_msgs.msg.Lift


class LiftState(BaseModel):
    lift_time: builtin_interfaces.msg.Time
    lift_name: str
    available_floors: list[str]
    current_floor: str
    destination_floor: str
    door_state: Annotated[int, Field(ge=0, le=255)]
    motion_state: Annotated[int, Field(ge=0, le=255)]
    available_modes: list[int]
    current_mode: Annotated[int, Field(ge=0, le=255)]
    session_id: str

    @staticmethod
    def from_tortoise(tortoise: ttm.LiftState) -> "LiftState":
        return LiftState.model_validate(tortoise.data)


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
    additional_session_ids: list[str] = Field(
        default=[],
        description="By default the node name of the API server is used, this field allows publishing the same request to additional session IDs",  # pylint: disable=line-too-long
    )
