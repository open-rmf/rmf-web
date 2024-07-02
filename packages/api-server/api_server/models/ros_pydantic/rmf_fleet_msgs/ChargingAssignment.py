# This is a generated file, do not edit

from typing import List

import pydantic


class ChargingAssignment(pydantic.BaseModel):
    robot_name: str = ""  # string
    waypoint_name: str = ""  # string
    mode: pydantic.conint(ge=0, le=255) = 0  # uint8

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "robot_name",
                "waypoint_name",
                "mode",
            ],
        }


# string robot_name
# string waypoint_name
# uint8 mode
#
# uint8 MODE_CHARGE = 0
# uint8 MODE_WAIT = 1
