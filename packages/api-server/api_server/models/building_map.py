from pydantic import BaseModel
from rmf_building_map_msgs.msg import BuildingMap as RmfBuildingMap


class BuildingMap(BaseModel, RmfBuildingMap):
    pass
