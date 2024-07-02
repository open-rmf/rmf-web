from typing import cast

from pydantic import BaseModel

from . import tortoise_models as ttm
from .ros_pydantic import rmf_ingestor_msgs


class Ingestor(BaseModel):
    guid: str


class IngestorState(rmf_ingestor_msgs.IngestorState):
    @staticmethod
    def from_tortoise(tortoise: ttm.IngestorState) -> "IngestorState":
        return IngestorState(**cast(dict, tortoise.data))
