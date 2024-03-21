# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.Location import Location


class DockParameter(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    start: str  # string
    finish: str  # string
    path: list[Location]  # rmf_fleet_msgs/Location


# # The name of the waypoint where the docking begins
# string start
#
# # The name of the waypoint where the docking ends
# string finish
#
# # The points in the docking path
# Location[] path
