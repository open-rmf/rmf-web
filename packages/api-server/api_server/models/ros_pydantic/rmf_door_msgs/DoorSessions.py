# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_door_msgs.Session import Session


class DoorSessions(pydantic.BaseModel):
    door_name: str
    sessions: List[Session]

    class Config:
        orm_mode = True

    def __init__(
        self,
        door_name: str = "",  # string
        sessions: List = None,  # rmf_door_msgs/Session
        **kwargs,
    ):
        super().__init__(
            door_name=door_name,
            sessions=sessions or [],
            **kwargs,
        )


#
# string door_name
# Session[] sessions
