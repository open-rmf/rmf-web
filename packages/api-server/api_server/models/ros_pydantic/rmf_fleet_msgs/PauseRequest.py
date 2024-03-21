# This is a generated file, do not edit

from typing import Annotated

import pydantic


class PauseRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str  # string
    robot_name: str  # string
    mode_request_id: Annotated[
        int, pydantic.Field(ge=0, le=18446744073709551615)
    ]  # uint64
    type: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32
    at_checkpoint: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32


# string fleet_name
# string robot_name
# uint64 mode_request_id
#
# uint32 TYPE_PAUSE_IMMEDIATELY=0
# uint32 TYPE_PAUSE_AT_CHECKPOINT=1
# uint32 TYPE_RESUME=2
# uint32 type
#
# uint32 at_checkpoint
