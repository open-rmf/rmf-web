# This is a generated file, do not edit

from typing import List

import pydantic


class LaneRequest(pydantic.BaseModel):
    fleet_name: str
    open_lanes: List[pydantic.conint(ge=0, le=18446744073709551615)]
    close_lanes: List[pydantic.conint(ge=0, le=18446744073709551615)]

    class Config:
        orm_mode = True

    def __init__(
        self,
        fleet_name: str = "",  # string
        open_lanes: List = None,  # uint64
        close_lanes: List = None,  # uint64
        **kwargs,
    ):
        super().__init__(
            fleet_name=fleet_name,
            open_lanes=open_lanes or [],
            close_lanes=close_lanes or [],
            **kwargs,
        )


#
# string fleet_name
# uint64[] open_lanes
# uint64[] close_lanes
