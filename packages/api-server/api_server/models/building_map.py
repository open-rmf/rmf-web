from typing import List

from api_server.models import tortoise_models as ttm
from api_server.models.ros_pydantic import rmf_building_map_msgs


class AffineImage(rmf_building_map_msgs.AffineImage):
    data: str


class Level(rmf_building_map_msgs.Level):
    images: List[AffineImage]


class BuildingMap(rmf_building_map_msgs.BuildingMap):
    levels: List[Level]

    @staticmethod
    def from_tortoise(tortoise: ttm.BuildingMap) -> "BuildingMap":
        return BuildingMap(**tortoise.data)

    async def save(self) -> None:
        await ttm.BuildingMap.update_or_create({"data": self.dict()}, id_=self.name)
