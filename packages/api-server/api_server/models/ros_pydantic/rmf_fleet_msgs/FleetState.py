# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.RobotState import RobotState


class FleetState(pydantic.BaseModel):
    name: str = ""  # string
    robots: List[RobotState] = []  # rmf_fleet_msgs/RobotState

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "name",
                "robots",
            ],
        }


# string name
# RobotState[] robots
