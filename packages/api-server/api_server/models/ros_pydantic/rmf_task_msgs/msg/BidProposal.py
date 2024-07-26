# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time


class BidProposal(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    expected_robot_name: str
    prev_cost: float
    new_cost: float
    finish_time: builtin_interfaces_msg_Time
