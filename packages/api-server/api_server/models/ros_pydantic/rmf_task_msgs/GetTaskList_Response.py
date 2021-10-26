# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.TaskSummary import TaskSummary


class GetTaskList_Response(pydantic.BaseModel):
    success: bool
    active_tasks: List[TaskSummary]
    terminated_tasks: List[TaskSummary]

    class Config:
        orm_mode = True

    def __init__(
        self,
        success: bool = False,  # bool
        active_tasks: List = None,  # rmf_task_msgs/TaskSummary
        terminated_tasks: List = None,  # rmf_task_msgs/TaskSummary
        **kwargs,
    ):
        super().__init__(
            success=success,
            active_tasks=active_tasks or [],
            terminated_tasks=terminated_tasks or [],
            **kwargs,
        )


#
#
# bool success
#
# TaskSummary[] active_tasks
# TaskSummary[] terminated_tasks
