# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Assignment(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    is_assigned: bool  # bool
    fleet_name: str  # string
    expected_robot_name: str  # string


# bool is_assigned
# string fleet_name
# string expected_robot_name
