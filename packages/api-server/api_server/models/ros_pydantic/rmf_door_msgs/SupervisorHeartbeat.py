# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_door_msgs.DoorSessions import DoorSessions


class SupervisorHeartbeat(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    all_sessions: list[DoorSessions]  # rmf_door_msgs/DoorSessions


#
# DoorSessions[] all_sessions
