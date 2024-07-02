# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_task_msgs.TaskDescription import TaskDescription


class TaskProfile(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    task_id: str  # string
    submission_time: Time  # builtin_interfaces/Time
    description: TaskDescription  # rmf_task_msgs/TaskDescription


# # Unique ID assigned to this task
# string task_id
#
# # Task submission time
# builtin_interfaces/Time submission_time
#
# # Details of the task
# TaskDescription description
