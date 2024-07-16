# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Duration import (
    Duration as builtin_interfaces_msg_Duration,
)
from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time


class ChargerState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    charger_time: builtin_interfaces_msg_Time
    state: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
    charger_name: str
    error_message: str
    request_id: str
    robot_fleet: str
    robot_name: str
    time_to_fully_charged: builtin_interfaces_msg_Duration
