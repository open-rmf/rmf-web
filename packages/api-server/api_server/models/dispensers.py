from pydantic import BaseModel

from .health import BasicHealth
from .ros_pydantic import rmf_dispenser_msgs

DispenserState = rmf_dispenser_msgs.DispenserState
DispenserHealth = BasicHealth


class Dispenser(BaseModel):
    guid: str
