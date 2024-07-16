# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time


class DispatchCommand(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    task_id: str
    dispatch_id: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
    timestamp: builtin_interfaces_msg_Time
    type: Annotated[int, pydantic.Field(ge=0, le=255)]
