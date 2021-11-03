# This is a generated file, do not edit

from typing import List

import pydantic


class CancelTask_Response(pydantic.BaseModel):
    success: bool = False  # bool
    message: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "success",
                "message",
            ],
        }


#
#
# # Confirmation that this service call is processed
# bool success
#
# # This will provide a verbose message regarding task cancellation
# string message
#
