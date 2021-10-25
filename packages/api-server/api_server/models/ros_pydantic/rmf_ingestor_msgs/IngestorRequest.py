# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_ingestor_msgs.IngestorRequestItem import IngestorRequestItem


class IngestorRequest(pydantic.BaseModel):
    time: Time
    request_guid: str
    target_guid: str
    transporter_type: str
    items: List[IngestorRequestItem]

    class Config:
        orm_mode = True

    def __init__(
        self,
        time: Time = Time(),  # builtin_interfaces/Time
        request_guid: str = "",  # string
        target_guid: str = "",  # string
        transporter_type: str = "",  # string
        items: List[IngestorRequestItem] = [],  # rmf_ingestor_msgs/IngestorRequestItem
    ):
        super().__init__(
            time=time,
            request_guid=request_guid,
            target_guid=target_guid,
            transporter_type=transporter_type,
            items=items,
        )


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
