# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.Location import Location


class PathRequest(pydantic.BaseModel):
    fleet_name: str = ""  # string
    robot_name: str = ""  # string
    path: List[Location] = []  # rmf_fleet_msgs/Location
    task_id: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "fleet_name",
                "robot_name",
                "path",
                "task_id",
            ],
        }


# string fleet_name
# string robot_name
# Location[] path
#
# # task_id must be copied into future RobotState messages
# string task_id
