# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.Location import Location


class DestinationRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str  # string
    robot_name: str  # string
    destination: Location  # rmf_fleet_msgs/Location
    task_id: str  # string


# string fleet_name
# string robot_name
# Location destination
#
# # task_id must be copied into future RobotState messages
# string task_id
