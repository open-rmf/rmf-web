# generated by datamodel-codegen:
#   filename:  cancel_task_request.json

from __future__ import annotations

from typing import List, Optional

from pydantic import BaseModel, Field
from typing_extensions import Literal


class CancelTaskRequest(BaseModel):
    type: Literal["cancel_task_request"] = Field(
        ..., description="Indicate that this is a task cancellation request"
    )
    task_id: str = Field(..., description="Specify the task ID to cancel")
    labels: Optional[List[str]] = Field(
        default=None,
        description="Labels to describe the purpose of the cancellation, items can be a single value like `dashboard` or a key-value pair like `app=dashboard`, in the case of a single value, it will be interpreted as a key-value pair with an empty string value.",
    )
