# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.SpeedLimitedLane import SpeedLimitedLane


class SpeedLimitRequest(pydantic.BaseModel):
    fleet_name: str = ""  # string
    speed_limits: List[SpeedLimitedLane] = []  # rmf_fleet_msgs/SpeedLimitedLane
    remove_limits: List[pydantic.conint(ge=0, le=18446744073709551615)] = []  # uint64

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "fleet_name",
                "speed_limits",
                "remove_limits",
            ],
        }


# # The name of the fleet
# string fleet_name
#
# # The lanes to impose speed limits upon.
# SpeedLimitedLane[] speed_limits
#
# # The indices of lanes to remove speed limits
# uint64[] remove_limits
