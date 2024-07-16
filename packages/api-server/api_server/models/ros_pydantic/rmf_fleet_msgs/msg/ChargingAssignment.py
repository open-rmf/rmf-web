# This is a generated file, do not edit

from typing import Annotated

import pydantic


class ChargingAssignment(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    robot_name: str
    waypoint_name: str
    mode: Annotated[int, pydantic.Field(ge=0, le=255)]
