# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.SpeedLimitedLane import SpeedLimitedLane


class LaneStates(pydantic.BaseModel):
    fleet_name: str = ""  # string
    closed_lanes: List[pydantic.conint(ge=0, le=18446744073709551615)] = []  # uint64
    speed_limits: List[SpeedLimitedLane] = []  # rmf_fleet_msgs/SpeedLimitedLane

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "fleet_name",
                "closed_lanes",
                "speed_limits",
            ],
        }


# # The name of the fleet with closed or speed limited lanes
# string fleet_name
#
# # The indices of the lanes that are currently closed
# uint64[] closed_lanes
#
# # Lanes that have speed limits
# SpeedLimitedLane[] speed_limits
