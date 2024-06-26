# This is a generated file, do not edit

from typing import List

import pydantic


class GetDispatchStates_Request(pydantic.BaseModel):
    task_ids: List[str] = []  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "task_ids",
            ],
        }


# # Query list of submitted tasks | Get service call
#
# # Input the generated task ID during submission
# # if empty, provide all Submitted Tasks
# string[] task_ids
#
