# This is a generated file, do not edit

from typing import List

import pydantic


class Place(pydantic.BaseModel):
    name: str
    x: float
    y: float
    yaw: float
    position_tolerance: float
    yaw_tolerance: float

    class Config:
        orm_mode = True

    def __init__(
        self,
        name: str = "",  # string
        x: float = 0,  # float32
        y: float = 0,  # float32
        yaw: float = 0,  # float32
        position_tolerance: float = 0,  # float32
        yaw_tolerance: float = 0,  # float32
    ):
        super().__init__(
            name=name,
            x=x,
            y=y,
            yaw=yaw,
            position_tolerance=position_tolerance,
            yaw_tolerance=yaw_tolerance,
        )


# string name
# float32 x
# float32 y
# float32 yaw
# float32 position_tolerance
# float32 yaw_tolerance
