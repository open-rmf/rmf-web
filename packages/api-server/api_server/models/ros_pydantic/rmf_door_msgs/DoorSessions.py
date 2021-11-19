# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_door_msgs.Session import Session


class DoorSessions(pydantic.BaseModel):
    door_name: str = ""  # string
    sessions: List[Session] = []  # rmf_door_msgs/Session

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "door_name",
                "sessions",
            ],
        }


#
# string door_name
# Session[] sessions
