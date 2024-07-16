# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..msg.TaskDescription import TaskDescription as rmf_task_msgs_msg_TaskDescription


class SubmitTask_Request(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    requester: str
    description: rmf_task_msgs_msg_TaskDescription
