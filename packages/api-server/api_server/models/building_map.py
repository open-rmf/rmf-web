from typing import List

from api_server.models.ros_pydantic import rmf_building_map_msgs


class AffineImage(rmf_building_map_msgs.AffineImage):
    data: str


class Level(rmf_building_map_msgs.Level):
    images: List[AffineImage]


class BuildingMap(rmf_building_map_msgs.BuildingMap):
    levels: List[Level]
