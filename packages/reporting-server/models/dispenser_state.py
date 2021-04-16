from enum import Enum

from tortoise import fields, models
from tortoise.contrib.pydantic import pydantic_model_creator

# {'log': 'INFO:app.BookKeeper.dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n', 'stream': 'stdout'}


class DispenserStateEnum(str, Enum):
    IDLE = "idle"
    BUSY = "busy"
    OFFLINE = "offline"


class DoorStateService:
    def get_state_name(self, state: int):
        if state == 0:
            return DispenserStateEnum.IDLE
        elif state == 1:
            return DispenserStateEnum.BUSY
        elif state == 2:
            return DispenserStateEnum.OFFLINE


class DispenserState(models.Model):
    id = fields.IntField(pk=True)
    state = fields.CharEnumField(DispenserStateEnum)
    payload = fields.JSONField()
    guid = fields.CharField(max_length=200)
    created = fields.DatetimeField(auto_now_add=True)

    service = DoorStateService()


DispenserState_Pydantic = pydantic_model_creator(DispenserState, name="DispenserState")
