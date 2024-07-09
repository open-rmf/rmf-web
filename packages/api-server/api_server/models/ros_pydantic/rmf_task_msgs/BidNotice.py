# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Duration import Duration


class BidNotice(pydantic.BaseModel):
    request: str = ""  # string
    task_id: str = ""  # string
    time_window: Duration = Duration()  # builtin_interfaces/Duration

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "request",
                "task_id",
                "time_window",
            ],
        }


# # This message is published by the Task Dispatcher node to notify all
# # Fleet Adapters to participate in a bidding process for a new task.
# # Fleet Adapters may then submit a BidProposal message with their best proposal
# # to accommodate the new task.
#
# # Details of the task request
# string request
#
# # The ID for this request
# string task_id
#
# # Duration for which the bidding is open
# builtin_interfaces/Duration time_window
