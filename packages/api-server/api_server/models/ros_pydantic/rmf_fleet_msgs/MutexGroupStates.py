# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.MutexGroupAssignment import MutexGroupAssignment


class MutexGroupStates(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    assignments: list[MutexGroupAssignment]  # rmf_fleet_msgs/MutexGroupAssignment


# # A map of all the current mutex group assignments
# MutexGroupAssignment[] assignments
