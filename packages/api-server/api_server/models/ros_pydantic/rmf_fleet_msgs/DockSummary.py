# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_fleet_msgs.Dock import Dock


class DockSummary(pydantic.BaseModel):
    docks: List[Dock] = []  # rmf_fleet_msgs/Dock

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "docks",
            ],
        }


# Dock[] docks
