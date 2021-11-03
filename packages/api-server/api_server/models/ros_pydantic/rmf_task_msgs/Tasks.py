# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.TaskSummary import TaskSummary


class Tasks(pydantic.BaseModel):
    tasks: List[TaskSummary] = []  # rmf_task_msgs/TaskSummary

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "tasks",
            ],
        }


# TaskSummary[] tasks
