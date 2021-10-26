# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.Param import Param


class GraphNode(pydantic.BaseModel):
    x: float
    y: float
    name: str
    params: List[Param]

    class Config:
        orm_mode = True

    def __init__(
        self,
        x: float = 0,  # float32
        y: float = 0,  # float32
        name: str = "",  # string
        params: List[Param] = [],  # rmf_building_map_msgs/Param
        **kwargs,
    ):
        super().__init__(
            x=x,
            y=y,
            name=name,
            params=params,
            **kwargs,
        )


# float32 x
# float32 y
# string name
# Param[] params
