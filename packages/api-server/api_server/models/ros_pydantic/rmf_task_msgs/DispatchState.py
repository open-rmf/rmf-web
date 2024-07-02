# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.Assignment import Assignment


class DispatchState(pydantic.BaseModel):
    task_id: str = ""  # string
    status: pydantic.conint(ge=-128, le=127) = 0  # int8
    assignment: Assignment = Assignment()  # rmf_task_msgs/Assignment
    errors: List[str] = []  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "task_id",
                "status",
                "assignment",
                "errors",
            ],
        }


#
# uint8 STATUS_UNINITIALIZED = 0
# uint8 STATUS_QUEUED = 1
# uint8 STATUS_SELECTED = 2
# uint8 STATUS_DISPATCHED = 3
# uint8 STATUS_FAILED_TO_ASSIGN = 4
# uint8 STATUS_CANCELED_IN_FLIGHT = 5
#
# string task_id
# int8 status
# Assignment assignment
# string[] errors
