# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .TaskSummary import TaskSummary as rmf_task_msgs_msg_TaskSummary


class Tasks(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    tasks: list[rmf_task_msgs_msg_TaskSummary]
