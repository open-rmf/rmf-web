from enum import Enum

from tortoise import fields, models
from tortoise.contrib.pydantic import pydantic_model_creator

from .door_state import DoorStateEnum


class LiftStateEnum(str, Enum):
    MODE_HUMAN = "human"
    MODE_AGV = "avg"
    MODE_UNKNOWN = "unknown"
    MODE_FIRE = "fire"
    MODE_EMERGENCY = "emergency"
    MODE_OFFLINE = "offline"


# FIXME: add all lift states
class LiftMotionStateEnum(str, Enum):
    MOTION_STOPPED = "stopped"


class LiftStateService:
    def get_state_name(self, state: int):
        if state == 0:
            return LiftStateEnum.MODE_HUMAN
        elif state == 1:
            return LiftStateEnum.MODE_AGV
        elif state == 2:
            return LiftStateEnum.MODE_UNKNOWN
        elif state == 3:
            return LiftStateEnum.MODE_FIRE
        elif state == 4:
            return LiftStateEnum.MODE_EMERGENCY
        elif state == 5:
            return LiftStateEnum.MODE_OFFLINE

    def get_motion_state_name(self, state: int):
        if state == 0:
            return LiftMotionStateEnum.MOTION_STOPPED
        elif state == 1:
            return LiftMotionStateEnum.MOTION_STOPPED
        elif state == 2:
            pass
        elif state == 3:
            pass


class LiftState(models.Model):
    id = fields.IntField(pk=True)
    door_state: DoorStateEnum = fields.CharEnumField(
        DoorStateEnum, default=DoorStateEnum.UNKNOWN
    )
    state: LiftStateEnum = fields.CharEnumField(
        LiftStateEnum, default=LiftStateEnum.MODE_UNKNOWN
    )
    destination_floor = fields.CharField(max_length=20)
    motion_state: LiftMotionStateEnum = fields.CharEnumField(
        LiftMotionStateEnum, default=LiftMotionStateEnum.MOTION_STOPPED
    )
    payload = fields.JSONField()
    current_floor = fields.CharField(max_length=20)
    session_id = fields.CharField(max_length=200)
    created = fields.DatetimeField(auto_now_add=True)

    service = LiftStateService()


LiftState_Pydantic = pydantic_model_creator(LiftState, name="LiftStates")
