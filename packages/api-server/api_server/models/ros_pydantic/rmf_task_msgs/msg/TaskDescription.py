# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ...builtin_interfaces.msg.Time import Time as builtin_interfaces_msg_Time
from .Clean import Clean as rmf_task_msgs_msg_Clean
from .Delivery import Delivery as rmf_task_msgs_msg_Delivery
from .Loop import Loop as rmf_task_msgs_msg_Loop
from .Priority import Priority as rmf_task_msgs_msg_Priority
from .Station import Station as rmf_task_msgs_msg_Station
from .TaskType import TaskType as rmf_task_msgs_msg_TaskType


class TaskDescription(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    start_time: builtin_interfaces_msg_Time
    priority: rmf_task_msgs_msg_Priority
    task_type: rmf_task_msgs_msg_TaskType
    station: rmf_task_msgs_msg_Station
    loop: rmf_task_msgs_msg_Loop
    delivery: rmf_task_msgs_msg_Delivery
    clean: rmf_task_msgs_msg_Clean
