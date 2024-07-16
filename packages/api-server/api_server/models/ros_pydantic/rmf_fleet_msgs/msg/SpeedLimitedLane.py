# This is a generated file, do not edit

from typing import Annotated

import pydantic


class SpeedLimitedLane(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    lane_index: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
    speed_limit: float
