# This is a generated file, do not edit

from typing import List

import pydantic


class SubmitTask_Response(pydantic.BaseModel):
    success: bool = False  # bool
    task_id: str = ""  # string
    message: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "success",
                "task_id",
                "message",
            ],
        }


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
