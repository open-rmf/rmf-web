from typing import Any, List, cast

from api_server.models.doors import Door
from api_server.models.lifts import Lift
from api_server.models.ros_pydantic.rmf_building_map_msgs.Graph import Graph
from api_server.models.ros_pydantic.rmf_building_map_msgs.Place import Place

from . import tortoise_models as ttm
from .ros_pydantic import rmf_building_map_msgs


class AffineImage(rmf_building_map_msgs.AffineImage):
    data: str

    def __init__(
        self,
        name: str = "",  # string
        x_offset: float = 0,  # float32
        y_offset: float = 0,  # float32
        yaw: float = 0,  # float32
        scale: float = 0,  # float32
        encoding: str = "",  # string
        data: str = "",
        **kwargs,
    ):
        super().__init__(
            name=name,
            x_offset=x_offset,
            y_offset=y_offset,
            yaw=yaw,
            scale=scale,
            encoding=encoding,
            data=cast(Any, data),
            **kwargs,
        )


class Level(rmf_building_map_msgs.Level):
    images: List[AffineImage]

    def __init__(
        self,
        name: str = "",  # string
        elevation: float = 0,  # float32
        images: List[AffineImage] = None,  # rmf_building_map_msgs/AffineImage
        places: List[Place] = None,  # rmf_building_map_msgs/Place
        doors: List[Door] = None,  # rmf_building_map_msgs/Door
        nav_graphs: List[Graph] = None,  # rmf_building_map_msgs/Graph
        wall_graph: Graph = Graph(),  # rmf_building_map_msgs/Graph
        **kwargs,
    ):
        super().__init__(
            name=name,
            elevation=elevation,
            images=images or [],
            places=places or [],
            doors=doors or [],
            nav_graphs=nav_graphs or [],
            wall_graph=wall_graph,
            **kwargs,
        )


class BuildingMap(rmf_building_map_msgs.BuildingMap):
    levels: List[Level]

    def __init__(
        self,
        name: str = "",  # string
        levels: List[Level] = None,  # rmf_building_map_msgs/Level
        lifts: List[Lift] = None,  # rmf_building_map_msgs/Lift
        **kwargs,
    ):
        super().__init__(
            name=name,
            levels=levels or [],
            lifts=lifts or [],
            **kwargs,
        )

    @staticmethod
    def from_tortoise(tortoise: ttm.BuildingMap) -> "BuildingMap":
        return BuildingMap(**tortoise.data)

    async def save(self) -> None:
        await ttm.BuildingMap.update_or_create({"data": self.dict()}, id_=self.name)
