# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time


class LiftRequest(pydantic.BaseModel):
    lift_name: str = ""  # string
    request_time: Time = Time()  # builtin_interfaces/Time
    session_id: str = ""  # string
    request_type: pydantic.conint(ge=0, le=255) = 0  # uint8
    destination_floor: str = ""  # string
    door_state: pydantic.conint(ge=0, le=255) = 0  # uint8

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "lift_name",
                "request_time",
                "session_id",
                "request_type",
                "destination_floor",
                "door_state",
            ],
        }


# string lift_name
# builtin_interfaces/Time request_time
#
# # session_id should be unique at least between different requesters.
# # For example, session_id could be the requester's node name.
# string session_id
#
# # AGV mode means that the doors are always open when the lift is stopped
# # Human mode means that LiftDoorRequest messages must be used to open/close
# # the doors explicitly, since they may "time out" and close automatically.
# uint8 request_type
# uint8 REQUEST_END_SESSION=0
# uint8 REQUEST_AGV_MODE=1
# uint8 REQUEST_HUMAN_MODE=2
#
# # The destination_floor must be one of the values returned in a LiftState.
# string destination_floor
#
# # Explicit door requests are necessary in "human" mode to open/close doors.
# # Door requests are not necessary in "AGV" mode, when the doors are always
# # held open when the lift cabin is stopped.
# uint8 door_state
# uint8 DOOR_CLOSED=0
# uint8 DOOR_OPEN=2
