# This is a generated file, do not edit

from typing import List

import pydantic


class GetTaskList_Request(pydantic.BaseModel):
    requester: str
    task_id: List[str]

    class Config:
        orm_mode = True

    def __init__(
        self,
        requester: str = "",  # string
        task_id: List[str] = [],  # string
    ):
        super().__init__(
            requester=requester,
            task_id=task_id,
        )


# # Query list of submitted tasks | Get service call
#
# # Identifier for who is requesting the service
# string requester
#
# # Input the generated task ID during submission
# # if empty, provide all Submitted Tasks
# string[] task_id
#
