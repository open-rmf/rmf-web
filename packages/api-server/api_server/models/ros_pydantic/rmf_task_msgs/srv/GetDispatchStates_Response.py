# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..msg.DispatchStates import DispatchStates as rmf_task_msgs_msg_DispatchStates


class GetDispatchStates_Response(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    success: bool
    states: rmf_task_msgs_msg_DispatchStates
