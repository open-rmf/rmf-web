from pydantic import BaseModel

from api_server.models.health import BasicHealth
from api_server.models.ros_pydantic import rmf_ingestor_msgs

IngestorState = rmf_ingestor_msgs.IngestorState
IngestorHealth = BasicHealth


class Ingestor(BaseModel):
    guid: str
