# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time
from .TaskDescription import TaskDescription as rmf_task_msgs_msg_TaskDescription


class TaskProfile(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    task_id: str
    submission_time: builtin_interfaces_msg_Time
    description: rmf_task_msgs_msg_TaskDescription
