# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.RobotState import RobotState


class FleetState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str  # string
    robots: list[RobotState]  # rmf_fleet_msgs/RobotState


# string name
# RobotState[] robots
