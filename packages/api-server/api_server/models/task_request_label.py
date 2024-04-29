from typing import Optional

import pydantic
from pydantic import BaseModel

# NOTE: This label model needs to exactly match the fields that are defined and
# populated by the dashboard. Any changes to either side will require syncing.


class TaskRequestLabelDescription(BaseModel):
    task_name: Optional[str]
    unix_millis_warn_time: Optional[str]
    pickup: Optional[str]
    destination: Optional[str]
    cart_id: Optional[str]

    @staticmethod
    def from_json_string(json_str: str) -> Optional["TaskRequestLabelDescription"]:
        try:
            return TaskRequestLabelDescription.parse_raw(json_str)
        except pydantic.error_wrappers.ValidationError:
            return None


class TaskRequestLabel(BaseModel):
    description: TaskRequestLabelDescription

    @staticmethod
    def from_json_string(json_str: str) -> Optional["TaskRequestLabel"]:
        try:
            return TaskRequestLabel.parse_raw(json_str)
        except pydantic.error_wrappers.ValidationError:
            return None
