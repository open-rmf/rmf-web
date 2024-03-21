# This is a generated file, do not edit

from typing import Annotated

import pydantic


class AffineImage(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str  # string
    x_offset: float  # float32
    y_offset: float  # float32
    yaw: float  # float32
    scale: float  # float32
    encoding: str  # string
    data: bytes  # uint8


# string name
# float32 x_offset
# float32 y_offset
# float32 yaw
# float32 scale
# string encoding
# uint8[] data
