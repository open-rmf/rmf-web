# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..builtin_interfaces.Duration import Duration


class BidNotice(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    request: str  # string
    task_id: str  # string
    time_window: Duration  # builtin_interfaces/Duration


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
