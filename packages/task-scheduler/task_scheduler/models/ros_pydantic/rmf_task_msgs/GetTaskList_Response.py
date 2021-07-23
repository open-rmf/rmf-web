# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.TaskSummary import TaskSummary


class GetTaskList_Response(pydantic.BaseModel):
    success: bool = False  # bool
    active_tasks: List[TaskSummary] = []  # rmf_task_msgs/TaskSummary
    terminated_tasks: List[TaskSummary] = []  # rmf_task_msgs/TaskSummary

    class Config:
        orm_mode = True


#
#
# bool success
#
# TaskSummary[] active_tasks
# TaskSummary[] terminated_tasks
