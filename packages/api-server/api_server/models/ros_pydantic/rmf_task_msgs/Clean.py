# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Clean(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    start_waypoint: str  # string


# # The name of the waypoint where the robot should begin its pre-configured
# # cleaning job.
# string start_waypoint
#
