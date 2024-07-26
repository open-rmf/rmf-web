# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .ModeParameter import ModeParameter as rmf_fleet_msgs_msg_ModeParameter
from .RobotMode import RobotMode as rmf_fleet_msgs_msg_RobotMode


class ModeRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    robot_name: str
    mode: rmf_fleet_msgs_msg_RobotMode
    task_id: str
    parameters: list[rmf_fleet_msgs_msg_ModeParameter]
