# This is a generated file, do not edit

from typing import Annotated

import pydantic


class SpeedLimitedLane(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    lane_index: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]  # uint64
    speed_limit: float  # float64


# # The index of the lane with a speed limit
# uint64 lane_index
#
# # The imposed speed limit for the lane
# float64 speed_limit
