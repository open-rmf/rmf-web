# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.TaskDescription import TaskDescription


class SubmitTask_Request(pydantic.BaseModel):
    requester: str = ""  # string
    description: TaskDescription = TaskDescription()  # rmf_task_msgs/TaskDescription

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "requester",
                "description",
            ],
        }


# # Submit Task | POST service call
#
# # Identifier for who is requesting the service
# string requester
#
# # desciption of task
# TaskDescription description
#
