# This is a generated file, do not edit

from typing import Annotated

import pydantic


class DoorMode(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    value: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32


# # The DoorMode message captures the "mode" of an automatic door controller.
# # Most door controllers default to running in "closed" mode, and transition
# # through some sort of "moving" mode until reaching the "open" mode.
#
# uint32 value
#
# # "value" must be one of the following enumerations:
# uint32 MODE_CLOSED=0
# uint32 MODE_MOVING=1
# uint32 MODE_OPEN=2
# uint32 MODE_OFFLINE=3
# uint32 MODE_UNKNOWN=4
