# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.SpeedLimitedLane import SpeedLimitedLane


class SpeedLimitRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str  # string
    speed_limits: list[SpeedLimitedLane]  # rmf_fleet_msgs/SpeedLimitedLane
    remove_limits: list[
        Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
    ]  # uint64


# # The name of the fleet
# string fleet_name
#
# # The lanes to impose speed limits upon.
# SpeedLimitedLane[] speed_limits
#
# # The indices of lanes to remove speed limits
# uint64[] remove_limits
