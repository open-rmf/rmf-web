# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..builtin_interfaces.Time import Time


class LiftRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    lift_name: str  # string
    request_time: Time  # builtin_interfaces/Time
    session_id: str  # string
    request_type: Annotated[int, pydantic.Field(ge=0, le=255)]  # uint8
    destination_floor: str  # string
    door_state: Annotated[int, pydantic.Field(ge=0, le=255)]  # uint8


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
