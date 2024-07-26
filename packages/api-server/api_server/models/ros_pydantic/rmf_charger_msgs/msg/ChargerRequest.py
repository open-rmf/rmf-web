# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Duration import (
    Duration as builtin_interfaces_msg_Duration,
)


class ChargerRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    charger_name: str
    fleet_name: str
    robot_name: str
    start_timeout: builtin_interfaces_msg_Duration
    request_id: str
