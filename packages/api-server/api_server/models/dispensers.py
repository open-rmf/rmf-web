from pydantic import BaseModel

from . import tortoise_models as ttm
from .ros_pydantic import rmf_dispenser_msgs

DispenserState = rmf_dispenser_msgs.DispenserState
DispenserHealth = ttm.DispenserHealth


class Dispenser(BaseModel):
    guid: str
