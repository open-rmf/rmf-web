# This is a generated file, do not edit

from typing import List

import pydantic


class SpeedLimitedLane(pydantic.BaseModel):
    lane_index: pydantic.conint(ge=0, le=18446744073709551615) = 0  # uint64
    speed_limit: float = 0  # float64

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "lane_index",
                "speed_limit",
            ],
        }


# # The index of the lane with a speed limit
# uint64 lane_index
#
# # The imposed speed limit for the lane
# float64 speed_limit
