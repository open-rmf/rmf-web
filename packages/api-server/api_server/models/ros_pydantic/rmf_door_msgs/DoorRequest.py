# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_door_msgs.DoorMode import DoorMode


class DoorRequest(pydantic.BaseModel):
    request_time: Time
    requester_id: str
    door_name: str
    requested_mode: DoorMode

    class Config:
        orm_mode = True

    def __init__(
        self,
        request_time: Time = Time(),  # builtin_interfaces/Time
        requester_id: str = "",  # string
        door_name: str = "",  # string
        requested_mode: DoorMode = DoorMode(),  # rmf_door_msgs/DoorMode
    ):
        super().__init__(
            request_time=request_time,
            requester_id=requester_id,
            door_name=door_name,
            requested_mode=requested_mode,
        )


# builtin_interfaces/Time request_time
# string requester_id
# string door_name
# DoorMode requested_mode
