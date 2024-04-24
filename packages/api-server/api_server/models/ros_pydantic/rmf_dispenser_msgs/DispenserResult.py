# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..builtin_interfaces.Time import Time


class DispenserResult(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    time: Time  # builtin_interfaces/Time
    request_guid: str  # string
    source_guid: str  # string
    status: Annotated[int, pydantic.Field(ge=0, le=255)]  # uint8


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
