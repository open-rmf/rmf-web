from typing import Optional

import pydantic
from pydantic import BaseModel

# NOTE: This label model needs to exactly match the fields that are defined and
# populated by the dashboard. Any changes to either side will require syncing.


class TaskBookingLabelDescription(BaseModel):
    """
    This description holds several fields that could be useful for frontend
    dashboards when dispatching a task, to then be identified or rendered
    accordingly back on the same frontend.
    """

    task_definition_id: str
    unix_millis_warn_time: Optional[int]
    pickup: Optional[str]
    destination: Optional[str]
    cart_id: Optional[str]

    @staticmethod
    def from_json_string(json_str: str) -> Optional["TaskBookingLabelDescription"]:
        try:
            return TaskBookingLabelDescription.parse_raw(json_str)
        except pydantic.error_wrappers.ValidationError:
            return None


class TaskBookingLabel(BaseModel):
    """
    This label is to be populated by any frontend during a task dispatch, by
    being added to TaskRequest.labels, which in turn populates
    TaskState.booking.labels, and can be used to display relevant information
    needed for any frontends.
    """

    description: TaskBookingLabelDescription

    @staticmethod
    def from_json_string(json_str: str) -> Optional["TaskBookingLabel"]:
        try:
            return TaskBookingLabel.parse_raw(json_str)
        except pydantic.error_wrappers.ValidationError:
            return None
