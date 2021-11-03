# This is a generated file, do not edit

from typing import List

import pydantic


class ClosedLanes(pydantic.BaseModel):
    fleet_name: str = ""  # string
    closed_lanes: List[pydantic.conint(ge=0, le=18446744073709551615)] = []  # uint64

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "fleet_name",
                "closed_lanes",
            ],
        }


#
# string fleet_name
# uint64[] closed_lanes
