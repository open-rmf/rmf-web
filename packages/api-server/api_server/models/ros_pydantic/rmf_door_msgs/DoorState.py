# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_door_msgs.DoorMode import DoorMode


class DoorState(pydantic.BaseModel):
    door_time: Time = Time()  # builtin_interfaces/Time
    door_name: str = ""  # string
    current_mode: DoorMode = DoorMode()  # rmf_door_msgs/DoorMode

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "door_time",
                "door_name",
                "current_mode",
            ],
        }


# builtin_interfaces/Time door_time
# string door_name
# DoorMode current_mode
