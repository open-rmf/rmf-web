# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_task_msgs.TaskDescription import TaskDescription


class TaskProfile(pydantic.BaseModel):
    task_id: str
    submission_time: Time
    description: TaskDescription

    class Config:
        orm_mode = True

    def __init__(
        self,
        task_id: str = "",  # string
        submission_time: Time = Time(),  # builtin_interfaces/Time
        description: TaskDescription = TaskDescription(),  # rmf_task_msgs/TaskDescription
        **kwargs,
    ):
        super().__init__(
            task_id=task_id,
            submission_time=submission_time,
            description=description,
            **kwargs,
        )


# # Unique ID assigned to this task
# string task_id
#
# # Task submission time
# builtin_interfaces/Time submission_time
#
# # Details of the task
# TaskDescription description
