# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_task_msgs.DispatchRequest import DispatchRequest


class DispatchAck(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    dispatch_request: DispatchRequest  # rmf_task_msgs/DispatchRequest
    success: bool  # bool


# # This message is published by the fleet adapter in response to a
# # DispatchRequest message. It indicates whether the requested task addition or
# # cancellation was successful.
#
# # The DispatchRequest message received by the Fleet Adapter
# DispatchRequest dispatch_request
#
# # True if the addition or cancellation operation was successful
# bool success
