# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .GraphEdge import GraphEdge as rmf_building_map_msgs_msg_GraphEdge
from .GraphNode import GraphNode as rmf_building_map_msgs_msg_GraphNode
from .Param import Param as rmf_building_map_msgs_msg_Param


class Graph(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    vertices: list[rmf_building_map_msgs_msg_GraphNode]
    edges: list[rmf_building_map_msgs_msg_GraphEdge]
    params: list[rmf_building_map_msgs_msg_Param]
