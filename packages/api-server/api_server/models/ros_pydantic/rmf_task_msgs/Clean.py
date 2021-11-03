# This is a generated file, do not edit

from typing import List

import pydantic


class Clean(pydantic.BaseModel):
    start_waypoint: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "start_waypoint",
            ],
        }


# # The name of the waypoint where the robot should begin its pre-configured
# # cleaning job.
# string start_waypoint
#
