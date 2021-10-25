# This is a generated file, do not edit

from typing import List

import pydantic


class ReviveTask_Response(pydantic.BaseModel):
    success: bool

    class Config:
        orm_mode = True

    def __init__(
        self,
        success: bool = False,  # bool
    ):
        super().__init__(
            success=success,
        )


#
#
# # Confirmation that this service call is processed
# bool success
#
