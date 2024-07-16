# This is a generated file, do not edit

from typing import Annotated

import pydantic


class LiftClearance_Request(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    robot_name: str
    lift_name: str
