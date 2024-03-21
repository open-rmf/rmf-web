# This is a generated file, do not edit

from typing import Annotated

import pydantic


class SubmitTask_Response(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    success: bool  # bool
    task_id: str  # string
    message: str  # string


#
#
# # Confirmation that this service call is processed
# bool success
#
# # generated task ID by dispatcher node
# string task_id
#
# # This will provide a verbose message regarding task submission
# string message
#
