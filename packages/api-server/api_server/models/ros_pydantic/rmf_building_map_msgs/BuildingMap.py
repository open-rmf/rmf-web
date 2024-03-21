# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_building_map_msgs.Level import Level
from ..rmf_building_map_msgs.Lift import Lift


class BuildingMap(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str  # string
    levels: list[Level]  # rmf_building_map_msgs/Level
    lifts: list[Lift]  # rmf_building_map_msgs/Lift


# string name
# Level[] levels
# Lift[] lifts
