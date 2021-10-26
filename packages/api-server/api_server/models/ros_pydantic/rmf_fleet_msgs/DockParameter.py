# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.Location import Location


class DockParameter(pydantic.BaseModel):
    start: str
    finish: str
    path: List[Location]

    class Config:
        orm_mode = True

    def __init__(
        self,
        start: str = "",  # string
        finish: str = "",  # string
        path: List = None,  # rmf_fleet_msgs/Location
        **kwargs,
    ):
        super().__init__(
            start=start,
            finish=finish,
            path=path or [],
            **kwargs,
        )


# # The name of the waypoint where the docking begins
# string start
#
# # The name of the waypoint where the docking ends
# string finish
#
# # The points in the docking path
# Location[] path
