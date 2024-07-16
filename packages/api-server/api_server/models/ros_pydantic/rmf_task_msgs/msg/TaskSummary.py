# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time
from .TaskProfile import TaskProfile as rmf_task_msgs_msg_TaskProfile


class TaskSummary(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str
    task_id: str
    task_profile: rmf_task_msgs_msg_TaskProfile
    state: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
    status: str
    submission_time: builtin_interfaces_msg_Time
    start_time: builtin_interfaces_msg_Time
    end_time: builtin_interfaces_msg_Time
    robot_name: str
