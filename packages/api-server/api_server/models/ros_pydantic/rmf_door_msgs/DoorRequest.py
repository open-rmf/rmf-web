# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_door_msgs.DoorMode import DoorMode


class DoorRequest(pydantic.BaseModel):
    request_time: Time = Time()  # builtin_interfaces/Time
    requester_id: str = ""  # string
    door_name: str = ""  # string
    requested_mode: DoorMode = DoorMode()  # rmf_door_msgs/DoorMode

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "request_time",
                "requester_id",
                "door_name",
                "requested_mode",
            ],
        }


# builtin_interfaces/Time request_time
# string requester_id
# string door_name
# DoorMode requested_mode
