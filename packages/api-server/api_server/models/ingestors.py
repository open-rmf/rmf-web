from pydantic import BaseModel

from .ros_pydantic import rmf_ingestor_msgs

IngestorState = rmf_ingestor_msgs.msg.IngestorState


class Ingestor(BaseModel):
    guid: str
