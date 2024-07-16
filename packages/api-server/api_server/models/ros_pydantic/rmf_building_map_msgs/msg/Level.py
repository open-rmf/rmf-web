# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .AffineImage import AffineImage as rmf_building_map_msgs_msg_AffineImage
from .Door import Door as rmf_building_map_msgs_msg_Door
from .Graph import Graph as rmf_building_map_msgs_msg_Graph
from .Place import Place as rmf_building_map_msgs_msg_Place


class Level(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    elevation: float
    images: list[rmf_building_map_msgs_msg_AffineImage]
    places: list[rmf_building_map_msgs_msg_Place]
    doors: list[rmf_building_map_msgs_msg_Door]
    nav_graphs: list[rmf_building_map_msgs_msg_Graph]
    wall_graph: rmf_building_map_msgs_msg_Graph
