from pydantic import BaseModel

from api_server.models.health import BasicHealth
from api_server.models.ros_pydantic import rmf_dispenser_msgs

DispenserState = rmf_dispenser_msgs.DispenserState
DispenserHealth = BasicHealth


class Dispenser(BaseModel):
    guid: str
