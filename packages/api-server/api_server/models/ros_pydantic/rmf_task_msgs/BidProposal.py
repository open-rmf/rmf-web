# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time
from ..rmf_task_msgs.TaskProfile import TaskProfile


class BidProposal(pydantic.BaseModel):
    fleet_name: str = ""  # string
    task_profile: TaskProfile = TaskProfile()  # rmf_task_msgs/TaskProfile
    prev_cost: float = 0  # float64
    new_cost: float = 0  # float64
    finish_time: Time = Time()  # builtin_interfaces/Time
    robot_name: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "fleet_name",
                "task_profile",
                "prev_cost",
                "new_cost",
                "finish_time",
                "robot_name",
            ],
        }


# # This message is published by a Fleet Adapter in response to a BidNotice
# # message.
#
# # The name of the Fleet Adapter publishing this message
# string fleet_name
#
# # Details of the task to accommodate. This should math the TaskProfile in the
# # BidNotice
# TaskProfile task_profile
#
# # The overall cost of task assignments prior to accommodating the new task
# float64 prev_cost
#
# # The overall cost of task assignments after accommodating the new task
# float64 new_cost
#
# # The estimated finish time of the new task
# builtin_interfaces/Time finish_time
#
# # The name of the robot in the fleet which will potentially execute the task
# string robot_name
