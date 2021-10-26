# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.BehaviorParameter import BehaviorParameter


class Behavior(pydantic.BaseModel):
    name: str
    parameters: List[BehaviorParameter]

    class Config:
        orm_mode = True

    def __init__(
        self,
        name: str = "",  # string
        parameters: List[BehaviorParameter] = [],  # rmf_task_msgs/BehaviorParameter
        **kwargs,
    ):
        super().__init__(
            name=name,
            parameters=parameters,
            **kwargs,
        )


# string name
# BehaviorParameter[] parameters
