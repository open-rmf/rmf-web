# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time


class Location(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    t: builtin_interfaces_msg_Time
    x: float
    y: float
    yaw: float
    obey_approach_speed_limit: bool
    approach_speed_limit: float
    level_name: str
    index: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
