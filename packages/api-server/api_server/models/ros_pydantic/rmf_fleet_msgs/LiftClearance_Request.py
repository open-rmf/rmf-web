# This is a generated file, do not edit

from typing import List

import pydantic


class LiftClearance_Request(pydantic.BaseModel):
    robot_name: str
    lift_name: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        robot_name: str = "",  # string
        lift_name: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            robot_name=robot_name,
            lift_name=lift_name,
            **kwargs,
        )


#
# # Name of the robot that wants to enter a lift
# string robot_name
#
# # Name of the lift that the robot wants to enter
# string lift_name
#
