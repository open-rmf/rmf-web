# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_task_msgs.TaskDescription import TaskDescription


class SubmitTask_Request(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    requester: str  # string
    description: TaskDescription  # rmf_task_msgs/TaskDescription


# # Submit Task | POST service call
#
# # Identifier for who is requesting the service
# string requester
#
# # desciption of task
# TaskDescription description
#
