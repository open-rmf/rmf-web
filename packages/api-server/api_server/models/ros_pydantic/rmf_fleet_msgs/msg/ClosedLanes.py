# This is a generated file, do not edit

from typing import Annotated

import pydantic


class ClosedLanes(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    closed_lanes: list[Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]]
