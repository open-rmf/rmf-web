# This is a generated file, do not edit

from typing import Annotated

import pydantic


class LaneRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    open_lanes: list[Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]]
    close_lanes: list[Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]]
