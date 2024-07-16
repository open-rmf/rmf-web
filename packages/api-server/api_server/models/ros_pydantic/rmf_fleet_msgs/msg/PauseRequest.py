# This is a generated file, do not edit

from typing import Annotated

import pydantic


class PauseRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    robot_name: str
    mode_request_id: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
    type: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
    at_checkpoint: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
