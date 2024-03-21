# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_door_msgs.DoorMode import DoorMode


class DoorState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    door_time: Time  # builtin_interfaces/Time
    door_name: str  # string
    current_mode: DoorMode  # rmf_door_msgs/DoorMode


# builtin_interfaces/Time door_time
# string door_name
# DoorMode current_mode
