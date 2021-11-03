# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.Param import Param


class GraphNode(pydantic.BaseModel):
    x: float = 0  # float32
    y: float = 0  # float32
    name: str = ""  # string
    params: List[Param] = []  # rmf_building_map_msgs/Param

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "x",
                "y",
                "name",
                "params",
            ],
        }


# float32 x
# float32 y
# string name
# Param[] params
