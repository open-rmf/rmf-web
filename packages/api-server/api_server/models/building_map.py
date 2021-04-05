from pydantic import BaseModel


class BuildingMap(BaseModel):
    """
    Schema is based on rmf building map, but with the image data changed to an url.
    See https://github.com/open-rmf/rmf_building_map_msgs/blob/main/rmf_building_map_msgs/msg/BuildingMap.msg
    """
