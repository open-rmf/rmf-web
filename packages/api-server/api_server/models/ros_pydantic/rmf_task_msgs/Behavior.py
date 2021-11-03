# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.BehaviorParameter import BehaviorParameter


class Behavior(pydantic.BaseModel):
    name: str = ""  # string
    parameters: List[BehaviorParameter] = []  # rmf_task_msgs/BehaviorParameter

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "name",
                "parameters",
            ],
        }


# string name
# BehaviorParameter[] parameters
