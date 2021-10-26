# This is a generated file, do not edit

from typing import List

import pydantic


class ClosedLanes(pydantic.BaseModel):
    fleet_name: str
    closed_lanes: List[pydantic.conint(ge=0, le=18446744073709551615)]

    class Config:
        orm_mode = True

    def __init__(
        self,
        fleet_name: str = "",  # string
        closed_lanes: List = None,  # uint64
        **kwargs,
    ):
        super().__init__(
            fleet_name=fleet_name,
            closed_lanes=closed_lanes or [],
            **kwargs,
        )


#
# string fleet_name
# uint64[] closed_lanes
