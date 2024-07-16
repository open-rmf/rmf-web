# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .MutexGroupAssignment import (
    MutexGroupAssignment as rmf_fleet_msgs_msg_MutexGroupAssignment,
)


class MutexGroupStates(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    assignments: list[rmf_fleet_msgs_msg_MutexGroupAssignment]
