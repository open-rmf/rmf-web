from pydantic import BaseModel

from .ros_pydantic import rmf_dispenser_msgs

DispenserState = rmf_dispenser_msgs.DispenserState


class Dispenser(BaseModel):
    guid: str
