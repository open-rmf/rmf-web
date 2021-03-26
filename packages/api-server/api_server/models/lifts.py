from enum import IntEnum

from pydantic import BaseModel
from rmf_lift_msgs.msg import LiftRequest as RmfLiftRequest


class LiftRequestModeEnum(IntEnum):
    END_SESSION = RmfLiftRequest.REQUEST_END_SESSION
    AGV = RmfLiftRequest.REQUEST_AGV_MODE
    HUMAN = RmfLiftRequest.REQUEST_HUMAN_MODE


class LiftRequestDoorModeEnum(IntEnum):
    CLOSED = RmfLiftRequest.DOOR_CLOSED
    OPEN = RmfLiftRequest.DOOR_OPEN


class LiftRequest(BaseModel):
    mode: LiftRequestModeEnum
    door_mode: LiftRequestDoorModeEnum
    destination: str
