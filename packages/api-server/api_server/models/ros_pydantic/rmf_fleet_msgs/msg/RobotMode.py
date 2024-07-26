# This is a generated file, do not edit

from typing import Annotated

import pydantic


class RobotMode(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    mode: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
    mode_request_id: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]
    performing_action: str
