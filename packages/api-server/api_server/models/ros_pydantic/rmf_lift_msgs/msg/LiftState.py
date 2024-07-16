# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time


class LiftState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    lift_time: builtin_interfaces_msg_Time
    lift_name: str
    available_floors: list[str]
    current_floor: str
    destination_floor: str
    door_state: Annotated[int, pydantic.Field(ge=0, le=255)]
    motion_state: Annotated[int, pydantic.Field(ge=0, le=255)]
    available_modes: list[Annotated[int, pydantic.Field(ge=0, le=255)]]
    current_mode: Annotated[int, pydantic.Field(ge=0, le=255)]
    session_id: str
