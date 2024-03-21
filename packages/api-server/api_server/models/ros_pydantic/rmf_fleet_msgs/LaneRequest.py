# This is a generated file, do not edit

from typing import Annotated

import pydantic


class LaneRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str  # string
    open_lanes: list[
        Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
    ]  # uint64
    close_lanes: list[
        Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
    ]  # uint64


#
# string fleet_name
# uint64[] open_lanes
# uint64[] close_lanes
