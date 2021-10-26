# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.Door import Door
from ..rmf_building_map_msgs.Graph import Graph


class Lift(pydantic.BaseModel):
    name: str
    levels: List[str]
    doors: List[Door]
    wall_graph: Graph
    ref_x: float
    ref_y: float
    ref_yaw: float
    width: float
    depth: float

    class Config:
        orm_mode = True

    def __init__(
        self,
        name: str = "",  # string
        levels: List[str] = [],  # string
        doors: List[Door] = [],  # rmf_building_map_msgs/Door
        wall_graph: Graph = Graph(),  # rmf_building_map_msgs/Graph
        ref_x: float = 0,  # float32
        ref_y: float = 0,  # float32
        ref_yaw: float = 0,  # float32
        width: float = 0,  # float32
        depth: float = 0,  # float32
        **kwargs,
    ):
        super().__init__(
            name=name,
            levels=levels,
            doors=doors,
            wall_graph=wall_graph,
            ref_x=ref_x,
            ref_y=ref_y,
            ref_yaw=ref_yaw,
            width=width,
            depth=depth,
            **kwargs,
        )


# string name
# string[] levels
# Door[] doors
# Graph wall_graph
#
# # (ref_x, ref_y, ref_yaw) is a "reference orientation" of the lift cabin
# # which can be used to align floors.
# float32 ref_x
# float32 ref_y
# float32 ref_yaw
#
# # width and depth of the cabin
# float32 width
# float32 depth
