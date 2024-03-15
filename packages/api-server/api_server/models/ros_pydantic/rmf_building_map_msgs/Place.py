# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Place(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str = ""  # string
    x: float = 0  # float32
    y: float = 0  # float32
    yaw: float = 0  # float32
    position_tolerance: float = 0  # float32
    yaw_tolerance: float = 0  # float32


# string name
# float32 x
# float32 y
# float32 yaw
# float32 position_tolerance
# float32 yaw_tolerance
