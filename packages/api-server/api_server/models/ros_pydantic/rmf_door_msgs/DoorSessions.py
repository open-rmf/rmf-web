# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_door_msgs.Session import Session


class DoorSessions(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    door_name: str  # string
    sessions: list[Session]  # rmf_door_msgs/Session


#
# string door_name
# Session[] sessions
