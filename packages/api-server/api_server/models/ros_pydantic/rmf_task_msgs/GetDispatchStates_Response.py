# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.DispatchStates import DispatchStates


class GetDispatchStates_Response(pydantic.BaseModel):
    success: bool = False  # bool
    states: DispatchStates = DispatchStates()  # rmf_task_msgs/DispatchStates

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "success",
                "states",
            ],
        }


#
#
# bool success
#
# DispatchStates states
