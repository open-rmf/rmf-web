# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.MutexGroupAssignment import MutexGroupAssignment


class MutexGroupStates(pydantic.BaseModel):
    assignments: List[MutexGroupAssignment] = []  # rmf_fleet_msgs/MutexGroupAssignment

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "assignments",
            ],
        }


# # A map of all the current mutex group assignments
# MutexGroupAssignment[] assignments
