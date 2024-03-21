# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_building_map_msgs.GraphEdge import GraphEdge
from ..rmf_building_map_msgs.GraphNode import GraphNode
from ..rmf_building_map_msgs.Param import Param


class Graph(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str  # string
    vertices: list[GraphNode]  # rmf_building_map_msgs/GraphNode
    edges: list[GraphEdge]  # rmf_building_map_msgs/GraphEdge
    params: list[Param]  # rmf_building_map_msgs/Param


# string name
# GraphNode[] vertices
# GraphEdge[] edges
# Param[] params
