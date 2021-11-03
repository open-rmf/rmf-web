# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.BuildingMap import BuildingMap


class GetBuildingMap_Response(pydantic.BaseModel):
    building_map: BuildingMap = BuildingMap()  # rmf_building_map_msgs/BuildingMap

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "building_map",
            ],
        }


#
# BuildingMap building_map
