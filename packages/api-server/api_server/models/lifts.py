# pylint: disable=line-too-long

from pydantic import BaseModel, Field


class LiftRequest(BaseModel):
    request_type: int = Field(
        ...,
        description="https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg",
    )
    door_mode: int = Field(
        ...,
        description="https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg",
    )
    destination: str
