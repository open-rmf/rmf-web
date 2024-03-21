# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..builtin_interfaces.Time import Time


class Location(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    t: Time  # builtin_interfaces/Time
    x: float  # float32
    y: float  # float32
    yaw: float  # float32
    obey_approach_speed_limit: bool  # bool
    approach_speed_limit: float  # float32
    level_name: str  # string
    index: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]  # uint64


# builtin_interfaces/Time t
# float32 x
# float32 y
# float32 yaw
#
# bool obey_approach_speed_limit false
# # Speed limit of the lane leading to this waypoint in m/s
# float32 approach_speed_limit
#
# string level_name
# uint64 index
