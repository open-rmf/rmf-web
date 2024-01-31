from typing import Optional, cast

from pydantic import BaseModel


class TaskQueueEntry(BaseModel):
    id: str
    assigned_to: str
    unix_millis_start_time: Optional[int]
    unix_millis_finish_time: Optional[int]
    status: Optional[str]
    unix_millis_request_time: Optional[int]
    requester: Optional[str]
    unix_millis_warn_time: Optional[int]
    pickup: Optional[str]
    destination: Optional[str]
