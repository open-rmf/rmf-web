# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time


class Session(pydantic.BaseModel):
    request_time: Time
    requester_id: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        request_time: Time = Time(),  # builtin_interfaces/Time
        requester_id: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            request_time=request_time,
            requester_id=requester_id,
            **kwargs,
        )


#
# builtin_interfaces/Time request_time
# string requester_id
