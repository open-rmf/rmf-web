# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_building_map_msgs.BuildingMap import BuildingMap


class GetBuildingMap_Response(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    building_map: BuildingMap  # rmf_building_map_msgs/BuildingMap


#
# BuildingMap building_map
