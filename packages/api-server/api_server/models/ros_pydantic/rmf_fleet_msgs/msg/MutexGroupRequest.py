# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time


class MutexGroupRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    group: str
    claimant: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
    claim_time: builtin_interfaces_msg_Time
    mode: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
