# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time


class DispenserResult(pydantic.BaseModel):
    time: Time = Time()  # builtin_interfaces/Time
    request_guid: str = ""  # string
    source_guid: str = ""  # string
    status: pydantic.conint(ge=0, le=255) = 0  # uint8

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "time",
                "request_guid",
                "source_guid",
                "status",
            ],
        }


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
