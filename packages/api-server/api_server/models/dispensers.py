from typing import cast

from pydantic import BaseModel

from . import tortoise_models as ttm
from .ros_pydantic import rmf_dispenser_msgs


class Dispenser(BaseModel):
    guid: str


class DispenserState(rmf_dispenser_msgs.DispenserState):
    @staticmethod
    def from_tortoise(tortoise: ttm.DispenserState) -> "DispenserState":
        return DispenserState(**cast(dict, tortoise.data))
