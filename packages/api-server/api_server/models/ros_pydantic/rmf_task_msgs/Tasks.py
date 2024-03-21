# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_task_msgs.TaskSummary import TaskSummary


class Tasks(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    tasks: list[TaskSummary]  # rmf_task_msgs/TaskSummary


# TaskSummary[] tasks
