# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time


class DispenserState(pydantic.BaseModel):
    time: Time = Time()  # builtin_interfaces/Time
    guid: str = ""  # string
    mode: pydantic.conint(ge=-2147483648, le=2147483647) = 0  # int32
    request_guid_queue: List[str] = []  # string
    seconds_remaining: float = 0  # float32

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "time",
                "guid",
                "mode",
                "request_guid_queue",
                "seconds_remaining",
            ],
        }


# builtin_interfaces/Time time
#
# # A unique ID for this workcell
# string guid
#
# # Different basic modes that the workcell could be in
# int32 mode
# int32 IDLE=0
# int32 BUSY=1
# int32 OFFLINE=2
#
# # Queued up requests that are being handled by this workcell
# string[] request_guid_queue
#
# # below are custom workcell message fields
# float32 seconds_remaining
