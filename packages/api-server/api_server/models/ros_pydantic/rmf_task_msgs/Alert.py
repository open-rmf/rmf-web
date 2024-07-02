# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_task_msgs.AlertParameter import AlertParameter


class Alert(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    id: str  # string
    title: str  # string
    subtitle: str  # string
    message: str  # string
    display: bool  # bool
    tier: Annotated[int, pydantic.Field(ge=0, le=255)]  # uint8
    responses_available: list[str]  # string
    alert_parameters: list[AlertParameter]  # rmf_task_msgs/AlertParameter
    task_id: str  # string


# # The unique ID which responses can reply to
# string id
#
# # Title, subtitle and message to be displayed on any frontend
# string title
# string subtitle
# string message
#
# # Whether this alert should be displayed on any frontend, default
# # as true
# bool display true
#
# # The severity tier of this alert
# uint8 tier
# uint8 TIER_INFO=0
# uint8 TIER_WARNING=1
# uint8 TIER_ERROR=2
#
# # Responses available for this alert. If no responses are expected
# # this field can be left empty
# string[] responses_available
#
# # Parameters that may be useful for custom interactions
# AlertParameter[] alert_parameters
#
# # The task ID that is involved in this alert. If no task is involved
# # this string can be left empty
# string task_id
