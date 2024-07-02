# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..builtin_interfaces.Time import Time


class LiftState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    lift_time: Time  # builtin_interfaces/Time
    lift_name: str  # string
    available_floors: list[str]  # string
    current_floor: str  # string
    destination_floor: str  # string
    door_state: Annotated[int, pydantic.Field(ge=0, le=255)]  # uint8
    motion_state: Annotated[int, pydantic.Field(ge=0, le=255)]  # uint8
    available_modes: bytes  # uint8
    current_mode: Annotated[int, pydantic.Field(ge=0, le=255)]  # uint8
    session_id: str  # string


# # lift_time records when the information in this message was generated
# builtin_interfaces/Time lift_time
#
# string lift_name
#
# string[] available_floors
# string current_floor
# string destination_floor
#
# uint8 door_state
# uint8 DOOR_CLOSED=0
# uint8 DOOR_MOVING=1
# uint8 DOOR_OPEN=2
#
# uint8 motion_state
# uint8 MOTION_STOPPED=0
# uint8 MOTION_UP=1
# uint8 MOTION_DOWN=2
# uint8 MOTION_UNKNOWN=3
#
# # We can only set human or agv mode, but we can read other modes: fire, etc.
# uint8[] available_modes
# uint8 current_mode
# uint8 MODE_UNKNOWN=0
# uint8 MODE_HUMAN=1
# uint8 MODE_AGV=2
# uint8 MODE_FIRE=3
# uint8 MODE_OFFLINE=4
# uint8 MODE_EMERGENCY=5
# # we can add more "read-only" modes as we come across more of them.
#
# # this field records the session_id that has been granted control of the lift
# # until it sends a request with a request_type of REQUEST_END_SESSION
# string session_id
