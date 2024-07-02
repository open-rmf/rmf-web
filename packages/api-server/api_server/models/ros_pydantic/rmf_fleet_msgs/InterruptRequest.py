# This is a generated file, do not edit

from typing import Annotated

import pydantic


class InterruptRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str  # string
    robot_name: str  # string
    interrupt_id: str  # string
    labels: list[str]  # string
    type: Annotated[int, pydantic.Field(ge=0, le=255)]  # uint8


# string fleet_name
# string robot_name
# string interrupt_id
# string[] labels
# uint8 type
#
# uint8 TYPE_INTERRUPT = 0
# uint8 TYPE_RESUME = 1
