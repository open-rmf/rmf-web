# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.ChargingAssignment import ChargingAssignment


class ChargingAssignments(pydantic.BaseModel):
    fleet_name: str = ""  # string
    assignments: List[ChargingAssignment] = []  # rmf_fleet_msgs/ChargingAssignment

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "fleet_name",
                "assignments",
            ],
        }


# string fleet_name
# ChargingAssignment[] assignments
