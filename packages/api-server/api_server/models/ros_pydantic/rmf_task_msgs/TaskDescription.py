# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_task_msgs.Clean import Clean
from ..rmf_task_msgs.Delivery import Delivery
from ..rmf_task_msgs.Loop import Loop
from ..rmf_task_msgs.Priority import Priority
from ..rmf_task_msgs.Station import Station
from ..rmf_task_msgs.TaskType import TaskType


class TaskDescription(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    start_time: Time  # builtin_interfaces/Time
    priority: Priority  # rmf_task_msgs/Priority
    task_type: TaskType  # rmf_task_msgs/TaskType
    station: Station  # rmf_task_msgs/Station
    loop: Loop  # rmf_task_msgs/Loop
    delivery: Delivery  # rmf_task_msgs/Delivery
    clean: Clean  # rmf_task_msgs/Clean


# # Desired start time of a task
# builtin_interfaces/Time start_time
#
# # Priority of the task
# Priority priority
#
# # Task type
# TaskType task_type
#
# # The corresponding field for the above TaskType should be populated
# Station station
# Loop loop
# Delivery delivery
# Clean clean
