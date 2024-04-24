# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_building_map_msgs.Door import Door
from ..rmf_building_map_msgs.Graph import Graph


class Lift(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str  # string
    levels: list[str]  # string
    doors: list[Door]  # rmf_building_map_msgs/Door
    wall_graph: Graph  # rmf_building_map_msgs/Graph
    ref_x: float  # float32
    ref_y: float  # float32
    ref_yaw: float  # float32
    width: float  # float32
    depth: float  # float32


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
