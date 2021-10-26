# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.TaskDescription import TaskDescription


class SubmitTask_Request(pydantic.BaseModel):
    requester: str
    description: TaskDescription

    class Config:
        orm_mode = True

    def __init__(
        self,
        requester: str = "",  # string
        description: TaskDescription = TaskDescription(),  # rmf_task_msgs/TaskDescription
        **kwargs,
    ):
        super().__init__(
            requester=requester,
            description=description,
            **kwargs,
        )


# # Submit Task | POST service call
#
# # Identifier for who is requesting the service
# string requester
#
# # desciption of task
# TaskDescription description
#
