# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.TaskSummary import TaskSummary


class Tasks(pydantic.BaseModel):
    tasks: List[TaskSummary]

    class Config:
        orm_mode = True

    def __init__(
        self,
        tasks: List = None,  # rmf_task_msgs/TaskSummary
        **kwargs,
    ):
        super().__init__(
            tasks=tasks or [],
            **kwargs,
        )


# TaskSummary[] tasks
