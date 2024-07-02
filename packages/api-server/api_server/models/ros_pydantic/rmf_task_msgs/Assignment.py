# This is a generated file, do not edit

from typing import List

import pydantic


class Assignment(pydantic.BaseModel):
    is_assigned: bool = False  # bool
    fleet_name: str = ""  # string
    expected_robot_name: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "is_assigned",
                "fleet_name",
                "expected_robot_name",
            ],
        }


# bool is_assigned
# string fleet_name
# string expected_robot_name
