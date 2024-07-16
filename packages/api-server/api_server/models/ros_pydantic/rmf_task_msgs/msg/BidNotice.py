# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Duration import (
    Duration as builtin_interfaces_msg_Duration,
)


class BidNotice(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    request: str
    task_id: str
    time_window: builtin_interfaces_msg_Duration
