# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .BehaviorParameter import BehaviorParameter as rmf_task_msgs_msg_BehaviorParameter


class Behavior(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    name: str
    parameters: list[rmf_task_msgs_msg_BehaviorParameter]
