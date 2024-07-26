# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .ChargingAssignment import (
    ChargingAssignment as rmf_fleet_msgs_msg_ChargingAssignment,
)


class ChargingAssignments(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    assignments: list[rmf_fleet_msgs_msg_ChargingAssignment]
