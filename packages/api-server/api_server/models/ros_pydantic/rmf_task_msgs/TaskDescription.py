# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_task_msgs.Clean import Clean
from ..rmf_task_msgs.Delivery import Delivery
from ..rmf_task_msgs.Loop import Loop
from ..rmf_task_msgs.Priority import Priority
from ..rmf_task_msgs.Station import Station
from ..rmf_task_msgs.TaskType import TaskType


class TaskDescription(pydantic.BaseModel):
    start_time: Time = Time()  # builtin_interfaces/Time
    priority: Priority = Priority()  # rmf_task_msgs/Priority
    task_type: TaskType = TaskType()  # rmf_task_msgs/TaskType
    station: Station = Station()  # rmf_task_msgs/Station
    loop: Loop = Loop()  # rmf_task_msgs/Loop
    delivery: Delivery = Delivery()  # rmf_task_msgs/Delivery
    clean: Clean = Clean()  # rmf_task_msgs/Clean

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "start_time",
                "priority",
                "task_type",
                "station",
                "loop",
                "delivery",
                "clean",
            ],
        }


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
