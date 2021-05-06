# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.Location import Location


class DestinationRequest(pydantic.BaseModel):
    fleet_name: str = ""  # string
    robot_name: str = ""  # string
    destination: Location = Location()  # rmf_fleet_msgs/Location
    task_id: str = ""  # string


# string fleet_name
# string robot_name
# Location destination
#
# # task_id must be copied into future RobotState messages
# string task_id
