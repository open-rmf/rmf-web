# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .AlertParameter import AlertParameter as rmf_task_msgs_msg_AlertParameter


class Alert(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    id: str
    title: str
    subtitle: str
    message: str
    display: bool
    tier: Annotated[int, pydantic.Field(ge=0, le=255)]
    responses_available: list[str]
    alert_parameters: list[rmf_task_msgs_msg_AlertParameter]
    task_id: str
