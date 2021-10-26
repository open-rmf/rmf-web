# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.TaskProfile import TaskProfile


class DispatchRequest(pydantic.BaseModel):
    fleet_name: str
    task_profile: TaskProfile
    method: pydantic.conint(ge=0, le=255)

    class Config:
        orm_mode = True

    def __init__(
        self,
        fleet_name: str = "",  # string
        task_profile: TaskProfile = TaskProfile(),  # rmf_task_msgs/TaskProfile
        method: int = 0,  # uint8
        **kwargs,
    ):
        super().__init__(
            fleet_name=fleet_name,
            task_profile=task_profile,
            method=method,
            **kwargs,
        )


# # This message is published by Task Dispatcher Node to either award or cancel a
# # task for a Fleet Adapter
#
# # The selected Fleet Adapter to award/cancel the task
# string fleet_name
#
# # The details of the task to be awarded or cancelled. This should match the
# # TaskProfile in the corresponding BidNotice message
# TaskProfile task_profile
#
# # Add or Cancel a task
# uint8 method
# uint8 ADD=1     # to add a task
# uint8 CANCEL=2  # to cancel and remove a task
