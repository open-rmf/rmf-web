# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time


class DispenserResult(pydantic.BaseModel):
    time: Time
    request_guid: str
    source_guid: str
    status: pydantic.conint(ge=0, le=255)

    class Config:
        orm_mode = True

    def __init__(
        self,
        time: Time = Time(),  # builtin_interfaces/Time
        request_guid: str = "",  # string
        source_guid: str = "",  # string
        status: pydantic.conint(ge=0, le=255) = 0,  # uint8
        **kwargs,
    ):
        super().__init__(
            time=time,
            request_guid=request_guid,
            source_guid=source_guid,
            status=status,
            **kwargs,
        )


# builtin_interfaces/Time time
#
# # A unique ID for the request which this result is for
# string request_guid
#
# # The unique ID of the workcell that this result was sent from
# string source_guid
#
# # Different basic result statuses
# uint8 status
# uint8 ACKNOWLEDGED=0
# uint8 SUCCESS=1
# uint8 FAILED=2
#
# # below are custom workcell message fields
