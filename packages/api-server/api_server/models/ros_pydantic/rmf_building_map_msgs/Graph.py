# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.GraphEdge import GraphEdge
from ..rmf_building_map_msgs.GraphNode import GraphNode
from ..rmf_building_map_msgs.Param import Param


class Graph(pydantic.BaseModel):
    name: str
    vertices: List[GraphNode]
    edges: List[GraphEdge]
    params: List[Param]

    class Config:
        orm_mode = True

    def __init__(
        self,
        name: str = "",  # string
        vertices: List[GraphNode] = [],  # rmf_building_map_msgs/GraphNode
        edges: List[GraphEdge] = [],  # rmf_building_map_msgs/GraphEdge
        params: List[Param] = [],  # rmf_building_map_msgs/Param
        **kwargs,
    ):
        super().__init__(
            name=name,
            vertices=vertices,
            edges=edges,
            params=params,
            **kwargs,
        )


# string name
# GraphNode[] vertices
# GraphEdge[] edges
# Param[] params
