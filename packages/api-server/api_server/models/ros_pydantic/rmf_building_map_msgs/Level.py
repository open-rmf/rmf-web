# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_building_map_msgs.AffineImage import AffineImage
from ..rmf_building_map_msgs.Door import Door
from ..rmf_building_map_msgs.Graph import Graph
from ..rmf_building_map_msgs.Place import Place


class Level(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str  # string
    elevation: float  # float32
    images: list[AffineImage]  # rmf_building_map_msgs/AffineImage
    places: list[Place]  # rmf_building_map_msgs/Place
    doors: list[Door]  # rmf_building_map_msgs/Door
    nav_graphs: list[Graph]  # rmf_building_map_msgs/Graph
    wall_graph: Graph  # rmf_building_map_msgs/Graph


# string name
# float32 elevation
# AffineImage[] images
# Place[] places
# Door[] doors
# Graph[] nav_graphs
# Graph wall_graph
