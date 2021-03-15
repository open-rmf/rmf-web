from building_map_msgs.msg import Door as RmfDoor

from .json_model import json_model


class Door(json_model(RmfDoor, lambda x: x.name)):
    pass
