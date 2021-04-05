from rmf_door_msgs.msg import DoorState as RmfDoorState

from .json_model import json_model


class DoorState(json_model(RmfDoorState, lambda x: x.door_name)):
    pass
