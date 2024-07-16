# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time
from .DispenserRequestItem import (
    DispenserRequestItem as rmf_dispenser_msgs_msg_DispenserRequestItem,
)


class DispenserRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    time: builtin_interfaces_msg_Time
    request_guid: str
    target_guid: str
    transporter_type: str
    items: list[rmf_dispenser_msgs_msg_DispenserRequestItem]
