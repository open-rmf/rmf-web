# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Door(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    v1_x: float
    v1_y: float
    v2_x: float
    v2_y: float
    door_type: Annotated[int, pydantic.Field(ge=0, le=255)]
    motion_range: float
    motion_direction: Annotated[int, pydantic.Field(ge=-2147483648, le=2147483647)]
