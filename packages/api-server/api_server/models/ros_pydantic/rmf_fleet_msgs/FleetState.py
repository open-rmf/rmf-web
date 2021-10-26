# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.RobotState import RobotState


class FleetState(pydantic.BaseModel):
    name: str
    robots: List[RobotState]

    class Config:
        orm_mode = True

    def __init__(
        self,
        name: str = "",  # string
        robots: List[RobotState] = [],  # rmf_fleet_msgs/RobotState
        **kwargs,
    ):
        super().__init__(
            name=name,
            robots=robots,
            **kwargs,
        )


# string name
# RobotState[] robots
