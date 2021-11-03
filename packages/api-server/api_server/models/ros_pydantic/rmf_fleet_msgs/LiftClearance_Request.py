# This is a generated file, do not edit

from typing import List

import pydantic


class LiftClearance_Request(pydantic.BaseModel):
    robot_name: str = ""  # string
    lift_name: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "robot_name",
                "lift_name",
            ],
        }


#
# # Name of the robot that wants to enter a lift
# string robot_name
#
# # Name of the lift that the robot wants to enter
# string lift_name
#
