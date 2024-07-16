# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .Param import Param as rmf_building_map_msgs_msg_Param


class GraphNode(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    x: float
    y: float
    name: str
    params: list[rmf_building_map_msgs_msg_Param]
