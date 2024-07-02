# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_fleet_msgs.ChargingAssignment import ChargingAssignment


class ChargingAssignments(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str  # string
    assignments: list[ChargingAssignment]  # rmf_fleet_msgs/ChargingAssignment


# string fleet_name
# ChargingAssignment[] assignments
