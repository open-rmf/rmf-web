# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..builtin_interfaces.Time import Time


class Session(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    request_time: Time  # builtin_interfaces/Time
    requester_id: str  # string


#
# builtin_interfaces/Time request_time
# string requester_id
