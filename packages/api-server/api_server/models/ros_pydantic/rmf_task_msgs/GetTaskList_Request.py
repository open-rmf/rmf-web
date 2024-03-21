# This is a generated file, do not edit

from typing import Annotated

import pydantic


class GetTaskList_Request(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    requester: str  # string
    task_id: list[str]  # string


# # Query list of submitted tasks | Get service call
#
# # Identifier for who is requesting the service
# string requester
#
# # Input the generated task ID during submission
# # if empty, provide all Submitted Tasks
# string[] task_id
#
