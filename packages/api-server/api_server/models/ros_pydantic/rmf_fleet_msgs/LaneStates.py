# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.SpeedLimitedLane import SpeedLimitedLane


class LaneStates(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str  # string
    closed_lanes: list[
        Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
    ]  # uint64
    speed_limits: list[SpeedLimitedLane]  # rmf_fleet_msgs/SpeedLimitedLane


# # The name of the fleet with closed or speed limited lanes
# string fleet_name
#
# # The indices of the lanes that are currently closed
# uint64[] closed_lanes
#
# # Lanes that have speed limits
# SpeedLimitedLane[] speed_limits
