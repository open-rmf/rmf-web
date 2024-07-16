# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Loop(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    task_id: str
    robot_type: str
    num_loops: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
    start_name: str
    finish_name: str
