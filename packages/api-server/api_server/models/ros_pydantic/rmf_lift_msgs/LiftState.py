# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time


class LiftState(pydantic.BaseModel):
    lift_time: Time
    lift_name: str
    available_floors: List[str]
    current_floor: str
    destination_floor: str
    door_state: pydantic.conint(ge=0, le=255)
    motion_state: pydantic.conint(ge=0, le=255)
    available_modes: bytes
    current_mode: pydantic.conint(ge=0, le=255)
    session_id: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        lift_time: Time = Time(),  # builtin_interfaces/Time
        lift_name: str = "",  # string
        available_floors: List = None,  # string
        current_floor: str = "",  # string
        destination_floor: str = "",  # string
        door_state: int = 0,  # uint8
        motion_state: int = 0,  # uint8
        available_modes: List = bytes(),  # uint8
        current_mode: int = 0,  # uint8
        session_id: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            lift_time=lift_time,
            lift_name=lift_name,
            available_floors=available_floors or [],
            current_floor=current_floor,
            destination_floor=destination_floor,
            door_state=door_state,
            motion_state=motion_state,
            available_modes=available_modes or [],
            current_mode=current_mode,
            session_id=session_id,
            **kwargs,
        )


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
