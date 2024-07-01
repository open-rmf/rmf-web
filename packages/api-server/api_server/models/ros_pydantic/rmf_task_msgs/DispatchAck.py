# This is a generated file, do not edit

from typing import List

import pydantic


class DispatchAck(pydantic.BaseModel):
    dispatch_id: pydantic.conint(ge=0, le=18446744073709551615) = 0  # uint64
    success: bool = False  # bool
    errors: List[str] = []  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "dispatch_id",
                "success",
                "errors",
            ],
        }


# # This message is published by the fleet adapter in response to a
# # DispatchRequest message. It indicates whether the requested task addition or
# # cancellation was successful.
#
# # The ID of the DispatchRequest that is being responded to
# uint64 dispatch_id
#
# # True if the addition or cancellation operation was successful
# bool success
#
# string[] errors
