# generated by datamodel-codegen:
#   filename:  task_request.json

from __future__ import annotations

from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field


class TaskRequest(BaseModel):
    unix_millis_earliest_start_time: Optional[int] = Field(
        default=None,
        description="(Optional) The earliest time that this task may start",
    )
    unix_millis_request_time: Optional[int] = Field(
        default=None, description="(Optional) The time that this request was initiated"
    )
    priority: Optional[Dict[str, Any]] = Field(
        default=None,
        description="(Optional) The priority of this task. This must match a priority schema supported by a fleet.",
    )
    category: str
    description: Any = Field(
        ...,
        description="A description of the task. This must match a schema supported by a fleet for the category of this task request.",
    )
    labels: Optional[List[str]] = Field(
        default=None,
        description="Labels to describe the purpose of the task dispatch request",
    )
    requester: Optional[str] = Field(
        default=None,
        description="(Optional) An identifier for the entity that requested this task",
    )
    fleet_name: Optional[str] = Field(
        default=None,
        description="(Optional) The name of the fleet that should perform this task. If specified, other fleets will not bid for this task.",
    )
