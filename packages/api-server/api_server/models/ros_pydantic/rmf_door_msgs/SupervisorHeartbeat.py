# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_door_msgs.DoorSessions import DoorSessions


class SupervisorHeartbeat(pydantic.BaseModel):
    all_sessions: List[DoorSessions] = []  # rmf_door_msgs/DoorSessions

    class Config:
        orm_mode = True


#
# DoorSessions[] all_sessions
