# This is a generated file, do not edit

from typing import Annotated

import pydantic


class LiftClearance_Request(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    robot_name: str  # string
    lift_name: str  # string


#
# # Name of the robot that wants to enter a lift
# string robot_name
#
# # Name of the lift that the robot wants to enter
# string lift_name
#
