# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time
from .DoorMode import DoorMode as rmf_door_msgs_msg_DoorMode


class DoorState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    door_time: builtin_interfaces_msg_Time
    door_name: str
    current_mode: rmf_door_msgs_msg_DoorMode
