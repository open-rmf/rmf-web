# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..msg.BuildingMap import BuildingMap as rmf_building_map_msgs_msg_BuildingMap


class GetBuildingMap_Response(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    building_map: rmf_building_map_msgs_msg_BuildingMap
