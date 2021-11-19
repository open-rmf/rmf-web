# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.Level import Level
from ..rmf_building_map_msgs.Lift import Lift


class BuildingMap(pydantic.BaseModel):
    name: str = ""  # string
    levels: List[Level] = []  # rmf_building_map_msgs/Level
    lifts: List[Lift] = []  # rmf_building_map_msgs/Lift

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "name",
                "levels",
                "lifts",
            ],
        }


# string name
# Level[] levels
# Lift[] lifts
