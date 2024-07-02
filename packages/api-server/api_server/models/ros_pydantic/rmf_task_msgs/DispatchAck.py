# This is a generated file, do not edit

from typing import Annotated

import pydantic


class DispatchAck(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    dispatch_id: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]  # uint64
    success: bool  # bool
    errors: list[str]  # string


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
