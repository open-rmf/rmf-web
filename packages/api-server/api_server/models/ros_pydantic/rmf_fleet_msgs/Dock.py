# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.DockParameter import DockParameter


class Dock(pydantic.BaseModel):
    fleet_name: str
    params: List[DockParameter]

    class Config:
        orm_mode = True

    def __init__(
        self,
        fleet_name: str = "",  # string
        params: List = None,  # rmf_fleet_msgs/DockParameter
        **kwargs,
    ):
        super().__init__(
            fleet_name=fleet_name,
            params=params or [],
            **kwargs,
        )


# string fleet_name
# DockParameter[] params
