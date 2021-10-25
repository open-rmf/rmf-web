# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.BuildingMap import BuildingMap


class GetBuildingMap_Response(pydantic.BaseModel):
    building_map: BuildingMap

    class Config:
        orm_mode = True

    def __init__(
        self,
        building_map: BuildingMap = BuildingMap(),  # rmf_building_map_msgs/BuildingMap
    ):
        super().__init__(
            building_map=building_map,
        )


#
# BuildingMap building_map
