# This is a generated file, do not edit

from typing import List

import pydantic


class LaneRequest(pydantic.BaseModel):
    fleet_name: str = ""  # string
    open_lanes: List[pydantic.conint(ge=0, le=18446744073709551615)] = []  # uint64
    close_lanes: List[pydantic.conint(ge=0, le=18446744073709551615)] = []  # uint64

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "fleet_name",
                "open_lanes",
                "close_lanes",
            ],
        }


#
# string fleet_name
# uint64[] open_lanes
# uint64[] close_lanes
