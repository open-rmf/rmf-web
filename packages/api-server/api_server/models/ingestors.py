from pydantic import BaseModel

from .health import BasicHealth
from .ros_pydantic import rmf_ingestor_msgs

IngestorState = rmf_ingestor_msgs.IngestorState
IngestorHealth = BasicHealth


class Ingestor(BaseModel):
    guid: str
