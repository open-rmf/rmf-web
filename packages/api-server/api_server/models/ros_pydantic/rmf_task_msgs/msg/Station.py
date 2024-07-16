# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Station(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    task_id: str
    robot_type: str
    place_name: str
