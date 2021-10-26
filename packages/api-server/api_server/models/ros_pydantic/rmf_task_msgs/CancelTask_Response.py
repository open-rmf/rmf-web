# This is a generated file, do not edit

from typing import List

import pydantic


class CancelTask_Response(pydantic.BaseModel):
    success: bool
    message: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        success: bool = False,  # bool
        message: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            success=success,
            message=message,
            **kwargs,
        )


#
#
# # Confirmation that this service call is processed
# bool success
#
# # This will provide a verbose message regarding task cancellation
# string message
#
