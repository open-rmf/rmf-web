# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.Dock import Dock


class DockSummary(pydantic.BaseModel):
    docks: List[Dock]

    class Config:
        orm_mode = True

    def __init__(
        self,
        docks: List = None,  # rmf_fleet_msgs/Dock
        **kwargs,
    ):
        super().__init__(
            docks=docks or [],
            **kwargs,
        )


# Dock[] docks
