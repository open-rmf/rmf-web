# This is a generated file, do not edit

from typing import List

import pydantic


class InterruptRequest(pydantic.BaseModel):
    fleet_name: str = ""  # string
    robot_name: str = ""  # string
    interrupt_id: str = ""  # string
    labels: List[str] = []  # string
    type: pydantic.conint(ge=0, le=255) = 0  # uint8

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "fleet_name",
                "robot_name",
                "interrupt_id",
                "labels",
                "type",
            ],
        }


# string fleet_name
# string robot_name
# string interrupt_id
# string[] labels
# uint8 type
#
# uint8 TYPE_INTERRUPT = 0
# uint8 TYPE_RESUME = 1
