# This is a generated file, do not edit

from typing import Annotated

import pydantic


class ChargingAssignment(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    robot_name: str  # string
    waypoint_name: str  # string
    mode: Annotated[int, pydantic.Field(ge=0, le=255)]  # uint8


# string robot_name
# string waypoint_name
# uint8 mode
#
# uint8 MODE_CHARGE = 0
# uint8 MODE_WAIT = 1
