# This is a generated file, do not edit

from typing import Annotated

import pydantic


class ReviveTask_Request(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    requester: str  # string
    task_id: str  # string


# # Revive a previously cancelled or failed task. This will reinitiate
# # a bidding sequence to reassign this task.
#
# # Identifier for who is requesting the service
# string requester
#
# # task that was previously cancelled or failed
# string task_id
#
