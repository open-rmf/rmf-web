# This is a generated file, do not edit

from typing import Annotated

import pydantic


class InterruptRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    robot_name: str
    interrupt_id: str
    labels: list[str]
    type: Annotated[int, pydantic.Field(ge=0, le=255)]
