# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.ModeParameter import ModeParameter
from ..rmf_fleet_msgs.RobotMode import RobotMode


class ModeRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str  # string
    robot_name: str  # string
    mode: RobotMode  # rmf_fleet_msgs/RobotMode
    task_id: str  # string
    parameters: list[ModeParameter]  # rmf_fleet_msgs/ModeParameter


# string fleet_name
# string robot_name
# RobotMode mode
#
# # task_id must be copied into future RobotState messages
# string task_id
#
# # Some mode changes require parameters. For example, the name of a dock.
# ModeParameter[] parameters
