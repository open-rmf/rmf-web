from pydantic import BaseModel

from . import tortoise_models as ttm
from .ros_pydantic import rmf_building_map_msgs


class AffineImage(BaseModel):
    name: str
    x_offset: float
    y_offset: float
    yaw: float
    scale: float
    encoding: str
    data: str


class Level(BaseModel):
    name: str
    elevation: float
    images: list[AffineImage]
    places: list[rmf_building_map_msgs.msg.Place]
    doors: list[rmf_building_map_msgs.msg.Door]
    nav_graphs: list[rmf_building_map_msgs.msg.Graph]
    wall_graph: rmf_building_map_msgs.msg.Graph


class BuildingMap(BaseModel):
    name: str
    levels: list[Level]
    lifts: list[rmf_building_map_msgs.msg.Lift]

    @staticmethod
    def from_tortoise(tortoise: ttm.BuildingMap) -> "BuildingMap":
        return BuildingMap(**dict(tortoise.data))


class FireAlarmTriggerState(BaseModel):
    unix_millis_time: int
    trigger: bool
