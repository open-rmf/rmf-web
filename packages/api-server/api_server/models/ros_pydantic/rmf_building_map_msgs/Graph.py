# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.GraphEdge import GraphEdge
from ..rmf_building_map_msgs.GraphNode import GraphNode
from ..rmf_building_map_msgs.Param import Param


class Graph(pydantic.BaseModel):
    name: str = ""  # string
    vertices: List[GraphNode] = []  # rmf_building_map_msgs/GraphNode
    edges: List[GraphEdge] = []  # rmf_building_map_msgs/GraphEdge
    params: List[Param] = []  # rmf_building_map_msgs/Param

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "name",
                "vertices",
                "edges",
                "params",
            ],
        }


# string name
# GraphNode[] vertices
# GraphEdge[] edges
# Param[] params
