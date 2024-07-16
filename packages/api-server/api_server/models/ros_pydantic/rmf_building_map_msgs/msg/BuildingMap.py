# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .Level import Level as rmf_building_map_msgs_msg_Level
from .Lift import Lift as rmf_building_map_msgs_msg_Lift


class BuildingMap(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    levels: list[rmf_building_map_msgs_msg_Level]
    lifts: list[rmf_building_map_msgs_msg_Lift]
