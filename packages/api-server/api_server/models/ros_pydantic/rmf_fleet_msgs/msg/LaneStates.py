# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .SpeedLimitedLane import SpeedLimitedLane as rmf_fleet_msgs_msg_SpeedLimitedLane


class LaneStates(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    closed_lanes: list[Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]]
    speed_limits: list[rmf_fleet_msgs_msg_SpeedLimitedLane]
