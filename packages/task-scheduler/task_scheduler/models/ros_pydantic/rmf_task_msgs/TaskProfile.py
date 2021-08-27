# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_task_msgs.TaskDescription import TaskDescription


class TaskProfile(pydantic.BaseModel):
    task_id: str = ""  # string
    submission_time: Time = Time()  # builtin_interfaces/Time
    description: TaskDescription = TaskDescription()  # rmf_task_msgs/TaskDescription

    class Config:
        orm_mode = True


# # Unique ID assigned to this task
# string task_id
#
# # Task submission time
# builtin_interfaces/Time submission_time
#
# # Details of the task
# TaskDescription description
