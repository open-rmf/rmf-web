# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.Location import Location


class DestinationRequest(pydantic.BaseModel):
    fleet_name: str
    robot_name: str
    destination: Location
    task_id: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        fleet_name: str = "",  # string
        robot_name: str = "",  # string
        destination: Location = Location(),  # rmf_fleet_msgs/Location
        task_id: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            fleet_name=fleet_name,
            robot_name=robot_name,
            destination=destination,
            task_id=task_id,
            **kwargs,
        )


# string fleet_name
# string robot_name
# Location destination
#
# # task_id must be copied into future RobotState messages
# string task_id
