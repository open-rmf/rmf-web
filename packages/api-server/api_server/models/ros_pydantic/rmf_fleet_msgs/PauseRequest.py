# This is a generated file, do not edit

from typing import List

import pydantic


class PauseRequest(pydantic.BaseModel):
    fleet_name: str
    robot_name: str
    mode_request_id: pydantic.conint(ge=0, le=18446744073709551615)
    type: pydantic.conint(ge=0, le=4294967295)
    at_checkpoint: pydantic.conint(ge=0, le=4294967295)

    class Config:
        orm_mode = True

    def __init__(
        self,
        fleet_name: str = "",  # string
        robot_name: str = "",  # string
        mode_request_id: pydantic.conint(ge=0, le=18446744073709551615) = 0,  # uint64
        type: pydantic.conint(ge=0, le=4294967295) = 0,  # uint32
        at_checkpoint: pydantic.conint(ge=0, le=4294967295) = 0,  # uint32
        **kwargs,
    ):
        super().__init__(
            fleet_name=fleet_name,
            robot_name=robot_name,
            mode_request_id=mode_request_id,
            type=type,
            at_checkpoint=at_checkpoint,
            **kwargs,
        )


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
