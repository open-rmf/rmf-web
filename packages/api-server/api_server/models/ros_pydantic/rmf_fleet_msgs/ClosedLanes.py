# This is a generated file, do not edit

from typing import List

import pydantic


class ClosedLanes(pydantic.BaseModel):
    fleet_name: str = ""  # string
    closed_lanes: List[pydantic.PositiveInt] = []  # uint64


#
# string fleet_name
# uint64[] closed_lanes
