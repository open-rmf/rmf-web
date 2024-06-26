# This is a generated file, do not edit

from typing import List

import pydantic


class MutexGroupManualRelease(pydantic.BaseModel):
    release_mutex_groups: List[str] = []  # string
    fleet: str = ""  # string
    robot: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "release_mutex_groups",
                "fleet",
                "robot",
            ],
        }


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
