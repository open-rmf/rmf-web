# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time


class Location(pydantic.BaseModel):
    t: Time
    x: float
    y: float
    yaw: float
    level_name: str
    index: pydantic.conint(ge=0, le=18446744073709551615)

    class Config:
        orm_mode = True

    def __init__(
        self,
        t: Time = Time(),  # builtin_interfaces/Time
        x: float = 0,  # float32
        y: float = 0,  # float32
        yaw: float = 0,  # float32
        level_name: str = "",  # string
        index: pydantic.conint(ge=0, le=18446744073709551615) = 0,  # uint64
    ):
        super().__init__(
            t=t,
            x=x,
            y=y,
            yaw=yaw,
            level_name=level_name,
            index=index,
        )


# builtin_interfaces/Time t
# float32 x
# float32 y
# float32 yaw
# string level_name
# uint64 index
