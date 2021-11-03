# This is a generated file, do not edit

from typing import List

import pydantic


class ReviveTask_Response(pydantic.BaseModel):
    success: bool = False  # bool

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "success",
            ],
        }


#
#
# # Confirmation that this service call is processed
# bool success
#
