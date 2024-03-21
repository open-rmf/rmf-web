# This is a generated file, do not edit

from typing import Annotated

import pydantic


class CancelTask_Request(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    requester: str  # string
    task_id: str  # string


# # Cancel Task | "Delete" service call
#
# # Identifier for who is requesting the service
# string requester
#
# # generated task ID by dispatcher node
# string task_id
#
