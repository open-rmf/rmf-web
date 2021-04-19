# This is a generated file, do not edit

from typing import List

import pydantic


class LaneRequest(pydantic.BaseModel):
    fleet_name: str = ""  # string
    open_lanes: List[pydantic.PositiveInt] = []  # uint64
    close_lanes: List[pydantic.PositiveInt] = []  # uint64


#
# string fleet_name
# uint64[] open_lanes
# uint64[] close_lanes
