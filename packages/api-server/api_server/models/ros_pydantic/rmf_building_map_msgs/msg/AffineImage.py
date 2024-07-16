# This is a generated file, do not edit

from typing import Annotated

import pydantic


class AffineImage(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    x_offset: float
    y_offset: float
    yaw: float
    scale: float
    encoding: str
    data: list[Annotated[int, pydantic.Field(ge=0, le=255)]]
