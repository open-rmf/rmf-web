# This is a generated file, do not edit

from typing import List

import pydantic


class ReviveTask_Request(pydantic.BaseModel):
    requester: str = ""  # string
    task_id: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "requester",
                "task_id",
            ],
        }


# # Revive a previously cancelled or failed task. This will reinitiate
# # a bidding sequence to reassign this task.
#
# # Identifier for who is requesting the service
# string requester
#
# # task that was previously cancelled or failed
# string task_id
#
