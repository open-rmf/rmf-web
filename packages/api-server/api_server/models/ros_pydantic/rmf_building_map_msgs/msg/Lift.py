# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .Door import Door as rmf_building_map_msgs_msg_Door
from .Graph import Graph as rmf_building_map_msgs_msg_Graph


class Lift(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    levels: list[str]
    doors: list[rmf_building_map_msgs_msg_Door]
    wall_graph: rmf_building_map_msgs_msg_Graph
    ref_x: float
    ref_y: float
    ref_yaw: float
    width: float
    depth: float
