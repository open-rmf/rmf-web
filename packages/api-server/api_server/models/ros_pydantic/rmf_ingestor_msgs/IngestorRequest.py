# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_ingestor_msgs.IngestorRequestItem import IngestorRequestItem


class IngestorRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    time: Time  # builtin_interfaces/Time
    request_guid: str  # string
    target_guid: str  # string
    transporter_type: str  # string
    items: list[IngestorRequestItem]  # rmf_ingestor_msgs/IngestorRequestItem


# builtin_interfaces/Time time
#
# # A unique ID for this request
# string request_guid
#
# # The unique name of the ingestor that this request is aimed at
# string target_guid
#
# # below are custom workcell message fields
# string transporter_type
# IngestorRequestItem[] items
