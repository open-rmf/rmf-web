# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .RobotState import RobotState as rmf_fleet_msgs_msg_RobotState


class FleetState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    robots: list[rmf_fleet_msgs_msg_RobotState]
