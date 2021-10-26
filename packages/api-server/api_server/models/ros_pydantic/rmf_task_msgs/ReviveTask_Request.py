# This is a generated file, do not edit

from typing import List

import pydantic


class ReviveTask_Request(pydantic.BaseModel):
    requester: str
    task_id: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        requester: str = "",  # string
        task_id: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            requester=requester,
            task_id=task_id,
            **kwargs,
        )


# # Revive a previously cancelled or failed task. This will reinitiate
# # a bidding sequence to reassign this task.
#
# # Identifier for who is requesting the service
# string requester
#
# # task that was previously cancelled or failed
# string task_id
#
