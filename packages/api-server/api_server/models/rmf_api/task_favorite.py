# generated by datamodel-codegen:
#   filename:  task_request.json

from __future__ import annotations

from typing import Any, Dict, Optional

from pydantic import BaseModel, Field


class TaskFavorite(BaseModel):
    name: str
    unix_millis_earliest_start_time: Optional[int] = Field(
        None, description="(Optional) The earliest time that this task may start"
    )
    priority: Optional[Dict[str, Any]] = Field(
        None,
        description="(Optional) The priority of this task.",
    )
    category: str
    description: Any = Field(
        ...,
        description="A description of the task. Task properties by category",
    )
