# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .DoorSessions import DoorSessions as rmf_door_msgs_msg_DoorSessions


class SupervisorHeartbeat(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    all_sessions: list[rmf_door_msgs_msg_DoorSessions]
