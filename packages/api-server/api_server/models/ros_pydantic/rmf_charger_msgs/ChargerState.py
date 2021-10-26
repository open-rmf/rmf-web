# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Duration import Duration
from ..builtin_interfaces.Time import Time


class ChargerState(pydantic.BaseModel):
    charger_time: Time
    state: pydantic.conint(ge=0, le=4294967295)
    charger_name: str
    error_message: str
    request_id: str
    robot_fleet: str
    robot_name: str
    time_to_fully_charged: Duration

    class Config:
        orm_mode = True

    def __init__(
        self,
        charger_time: Time = Time(),  # builtin_interfaces/Time
        state: pydantic.conint(ge=0, le=4294967295) = 0,  # uint32
        charger_name: str = "",  # string
        error_message: str = "",  # string
        request_id: str = "",  # string
        robot_fleet: str = "",  # string
        robot_name: str = "",  # string
        time_to_fully_charged: Duration = Duration(),  # builtin_interfaces/Duration
        **kwargs,
    ):
        super().__init__(
            charger_time=charger_time,
            state=state,
            charger_name=charger_name,
            error_message=error_message,
            request_id=request_id,
            robot_fleet=robot_fleet,
            robot_name=robot_name,
            time_to_fully_charged=time_to_fully_charged,
            **kwargs,
        )


# # Time when this state message was created
# builtin_interfaces/Time charger_time
#
# uint32 CHARGER_IDLE = 1      # Charger is not occupied
# uint32 CHARGER_ASSIGNED = 2  # Charger has been assigned a robot
# uint32 CHARGER_CHARGING = 3  # Charger is charging
# uint32 CHARGER_RELEASED = 4  # Charger has been disconnected from a robot
# uint32 CHARGER_ERROR = 200   # Error state, see error_message for info
#
# uint32 state  # One of the previously enumerated states
#
# # The charger name should be unique in the RMF system and
# # should match a charger name appearing in the traffic map
# string charger_name
#
# # The error_message field should be blank unless state is CHARGER_ERROR
# string error_message
#
# # The request_id field will be populated with the ID that started the
# # charging cycle if state is anything other than CHARGER_IDLE
# string request_id
#
# # The robot that is currently assigned to this charger (if any)
# string robot_fleet
# string robot_name
#
# # This contains the duration till the robot becomes fully charged.
# builtin_interfaces/Duration time_to_fully_charged
