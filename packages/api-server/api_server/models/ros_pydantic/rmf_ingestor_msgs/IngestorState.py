# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time


class IngestorState(pydantic.BaseModel):
    time: Time
    guid: str
    mode: pydantic.conint(ge=-2147483648, le=2147483647)
    request_guid_queue: List[str]
    seconds_remaining: float

    class Config:
        orm_mode = True

    def __init__(
        self,
        time: Time = Time(),  # builtin_interfaces/Time
        guid: str = "",  # string
        mode: int = 0,  # int32
        request_guid_queue: List = None,  # string
        seconds_remaining: float = 0,  # float32
        **kwargs,
    ):
        super().__init__(
            time=time,
            guid=guid,
            mode=mode,
            request_guid_queue=request_guid_queue or [],
            seconds_remaining=seconds_remaining,
            **kwargs,
        )


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
