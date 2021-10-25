# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.AffineImage import AffineImage
from ..rmf_building_map_msgs.Door import Door
from ..rmf_building_map_msgs.Graph import Graph
from ..rmf_building_map_msgs.Place import Place


class Level(pydantic.BaseModel):
    name: str
    elevation: float
    images: List[AffineImage]
    places: List[Place]
    doors: List[Door]
    nav_graphs: List[Graph]
    wall_graph: Graph

    class Config:
        orm_mode = True

    def __init__(
        self,
        name: str = "",  # string
        elevation: float = 0,  # float32
        images: List[AffineImage] = [],  # rmf_building_map_msgs/AffineImage
        places: List[Place] = [],  # rmf_building_map_msgs/Place
        doors: List[Door] = [],  # rmf_building_map_msgs/Door
        nav_graphs: List[Graph] = [],  # rmf_building_map_msgs/Graph
        wall_graph: Graph = Graph(),  # rmf_building_map_msgs/Graph
    ):
        super().__init__(
            name=name,
            elevation=elevation,
            images=images,
            places=places,
            doors=doors,
            nav_graphs=nav_graphs,
            wall_graph=wall_graph,
        )


# string name
# float32 elevation
# AffineImage[] images
# Place[] places
# Door[] doors
# Graph[] nav_graphs
# Graph wall_graph
