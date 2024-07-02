from . import tortoise_models as ttm
from .ros_pydantic import rmf_building_map_msgs


class AffineImage(rmf_building_map_msgs.AffineImage):
    data: str  # pyright: ignore [reportIncompatibleVariableOverride]


class Level(rmf_building_map_msgs.Level):
    images: list[AffineImage]  # pyright: ignore [reportIncompatibleVariableOverride]


class BuildingMap(rmf_building_map_msgs.BuildingMap):
    levels: list[Level]  # pyright: ignore [reportIncompatibleVariableOverride]

    @staticmethod
    def from_tortoise(tortoise: ttm.BuildingMap) -> "BuildingMap":
        return BuildingMap(**dict(tortoise.data))

    async def save(self) -> None:
        existing_maps = await ttm.BuildingMap.all()
        for m in existing_maps:
            if m.id_ != self.name:
                await m.delete()
        await ttm.BuildingMap.update_or_create(
            {"data": self.model_dump()}, id_=self.name
        )
