# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time


class Session(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    request_time: builtin_interfaces_msg_Time
    requester_id: str
