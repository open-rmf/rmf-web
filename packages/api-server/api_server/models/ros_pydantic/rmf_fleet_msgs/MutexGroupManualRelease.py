# This is a generated file, do not edit

from typing import Annotated

import pydantic


class MutexGroupManualRelease(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    release_mutex_groups: list[str]  # string
    fleet: str  # string
    robot: str  # string


# # This message allows operators to manually request that a robot release one or
# # more mutex groups that it is currently holding.
#
# # Name of the mutex groups to release
# string[] release_mutex_groups
#
# # The name of the fleet that the robot belongs to
# string fleet
#
# # The name of the robot that needs to release the mutex groups
# string robot
