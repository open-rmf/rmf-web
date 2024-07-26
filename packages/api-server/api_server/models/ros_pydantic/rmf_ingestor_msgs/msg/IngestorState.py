# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time


class IngestorState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    time: builtin_interfaces_msg_Time
    guid: str
    mode: Annotated[int, pydantic.Field(ge=-2147483648, le=2147483647)]
    request_guid_queue: list[str]
    seconds_remaining: float
