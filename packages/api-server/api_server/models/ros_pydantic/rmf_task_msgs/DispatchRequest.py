# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_task_msgs.TaskProfile import TaskProfile


class DispatchRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    fleet_name: str  # string
    task_profile: TaskProfile  # rmf_task_msgs/TaskProfile
    method: Annotated[int, pydantic.Field(ge=0, le=255)]  # uint8


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
