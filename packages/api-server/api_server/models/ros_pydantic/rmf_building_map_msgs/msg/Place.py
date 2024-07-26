# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Place(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    x: float
    y: float
    yaw: float
    position_tolerance: float
    yaw_tolerance: float
