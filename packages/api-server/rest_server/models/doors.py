from enum import IntEnum

from pydantic import BaseModel
from rmf_door_msgs.msg import DoorMode as RmfDoorMode


class DoorModeEnum(IntEnum):
    OPEN = RmfDoorMode.MODE_OPEN
    CLOSED = RmfDoorMode.MODE_CLOSED


class DoorMode(BaseModel):
    mode: DoorModeEnum
