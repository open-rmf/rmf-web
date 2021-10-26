# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_door_msgs.DoorSessions import DoorSessions


class SupervisorHeartbeat(pydantic.BaseModel):
    all_sessions: List[DoorSessions]

    class Config:
        orm_mode = True

    def __init__(
        self,
        all_sessions: List = None,  # rmf_door_msgs/DoorSessions
        **kwargs,
    ):
        super().__init__(
            all_sessions=all_sessions or [],
            **kwargs,
        )


#
# DoorSessions[] all_sessions
