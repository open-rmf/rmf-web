# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .Session import Session as rmf_door_msgs_msg_Session


class DoorSessions(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    door_name: str
    sessions: list[rmf_door_msgs_msg_Session]
