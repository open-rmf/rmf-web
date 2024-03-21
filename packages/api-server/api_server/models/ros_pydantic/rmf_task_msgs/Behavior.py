# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_task_msgs.BehaviorParameter import BehaviorParameter


class Behavior(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str  # string
    parameters: list[BehaviorParameter]  # rmf_task_msgs/BehaviorParameter


# string name
# BehaviorParameter[] parameters
