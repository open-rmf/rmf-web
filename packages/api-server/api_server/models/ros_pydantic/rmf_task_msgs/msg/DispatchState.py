# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .Assignment import Assignment as rmf_task_msgs_msg_Assignment


class DispatchState(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    task_id: str
    status: Annotated[int, pydantic.Field(ge=-128, le=127)]
    assignment: rmf_task_msgs_msg_Assignment
    errors: list[str]
