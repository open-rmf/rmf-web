# This is a generated file, do not edit

from typing import List

import pydantic


class SubmitTask_Response(pydantic.BaseModel):
    success: bool
    task_id: str
    message: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        success: bool = False,  # bool
        task_id: str = "",  # string
        message: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            success=success,
            task_id=task_id,
            message=message,
            **kwargs,
        )


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
