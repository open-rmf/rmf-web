# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_building_map_msgs.Param import Param


class GraphNode(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    x: float  # float32
    y: float  # float32
    name: str  # string
    params: list[Param]  # rmf_building_map_msgs/Param


# float32 x
# float32 y
# string name
# Param[] params
