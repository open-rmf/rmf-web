# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time
from .DoorMode import DoorMode as rmf_door_msgs_msg_DoorMode


class DoorRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    request_time: builtin_interfaces_msg_Time
    requester_id: str
    door_name: str
    requested_mode: rmf_door_msgs_msg_DoorMode
