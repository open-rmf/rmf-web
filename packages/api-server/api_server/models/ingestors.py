from pydantic import BaseModel

from . import tortoise_models as ttm
from .ros_pydantic import rmf_ingestor_msgs

IngestorState = rmf_ingestor_msgs.IngestorState
IngestorHealth = ttm.IngestorHealth


class Ingestor(BaseModel):
    guid: str
