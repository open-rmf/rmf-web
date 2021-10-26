# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.Level import Level
from ..rmf_building_map_msgs.Lift import Lift


class BuildingMap(pydantic.BaseModel):
    name: str
    levels: List[Level]
    lifts: List[Lift]

    class Config:
        orm_mode = True

    def __init__(
        self,
        name: str = "",  # string
        levels: List[Level] = [],  # rmf_building_map_msgs/Level
        lifts: List[Lift] = [],  # rmf_building_map_msgs/Lift
        **kwargs,
    ):
        super().__init__(
            name=name,
            levels=levels,
            lifts=lifts,
            **kwargs,
        )


# string name
# Level[] levels
# Lift[] lifts
