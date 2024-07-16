# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .DispatchState import DispatchState as rmf_task_msgs_msg_DispatchState


class DispatchStates(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    active: list[rmf_task_msgs_msg_DispatchState]
    finished: list[rmf_task_msgs_msg_DispatchState]
