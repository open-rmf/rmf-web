# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_task_msgs.TaskSummary import TaskSummary


class GetTaskList_Response(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    success: bool  # bool
    active_tasks: list[TaskSummary]  # rmf_task_msgs/TaskSummary
    terminated_tasks: list[TaskSummary]  # rmf_task_msgs/TaskSummary


#
#
# bool success
#
# TaskSummary[] active_tasks
# TaskSummary[] terminated_tasks
