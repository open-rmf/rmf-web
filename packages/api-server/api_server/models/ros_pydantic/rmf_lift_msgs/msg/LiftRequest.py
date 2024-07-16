# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time


class LiftRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    lift_name: str
    request_time: builtin_interfaces_msg_Time
    session_id: str
    request_type: Annotated[int, pydantic.Field(ge=0, le=255)]
    destination_floor: str
    door_state: Annotated[int, pydantic.Field(ge=0, le=255)]
