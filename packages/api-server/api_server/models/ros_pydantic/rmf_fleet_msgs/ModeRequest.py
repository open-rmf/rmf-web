# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.ModeParameter import ModeParameter
from ..rmf_fleet_msgs.RobotMode import RobotMode


class ModeRequest(pydantic.BaseModel):
    fleet_name: str
    robot_name: str
    mode: RobotMode
    task_id: str
    parameters: List[ModeParameter]

    class Config:
        orm_mode = True

    def __init__(
        self,
        fleet_name: str = "",  # string
        robot_name: str = "",  # string
        mode: RobotMode = RobotMode(),  # rmf_fleet_msgs/RobotMode
        task_id: str = "",  # string
        parameters: List = None,  # rmf_fleet_msgs/ModeParameter
        **kwargs,
    ):
        super().__init__(
            fleet_name=fleet_name,
            robot_name=robot_name,
            mode=mode,
            task_id=task_id,
            parameters=parameters or [],
            **kwargs,
        )


# string fleet_name
# string robot_name
# RobotMode mode
#
# # task_id must be copied into future RobotState messages
# string task_id
#
# # Some mode changes require parameters. For example, the name of a dock.
# ModeParameter[] parameters
