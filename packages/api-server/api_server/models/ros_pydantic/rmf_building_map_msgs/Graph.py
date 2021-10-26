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
        vertices: List = None,  # rmf_building_map_msgs/GraphNode
        edges: List = None,  # rmf_building_map_msgs/GraphEdge
        params: List = None,  # rmf_building_map_msgs/Param
        **kwargs,
    ):
        super().__init__(
            name=name,
            vertices=vertices or [],
            edges=edges or [],
            params=params or [],
            **kwargs,
        )


# string name
# GraphNode[] vertices
# GraphEdge[] edges
# Param[] params
