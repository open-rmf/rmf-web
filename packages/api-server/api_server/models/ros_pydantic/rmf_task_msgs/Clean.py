# This is a generated file, do not edit

from typing import List

import pydantic


class Clean(pydantic.BaseModel):
    start_waypoint: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        start_waypoint: str = "",  # string
    ):
        super().__init__(
            start_waypoint=start_waypoint,
        )


# # The name of the waypoint where the robot should begin its pre-configured
# # cleaning job.
# string start_waypoint
#
