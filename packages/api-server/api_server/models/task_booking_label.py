from typing import Optional

import pydantic
from pydantic import BaseModel


class TaskBookingLabel(BaseModel):
    """
    This label is to be populated by any frontend during a task dispatch, by
    being added to TaskRequest.labels, which in turn populates
    TaskState.booking.labels, and can be used to display relevant information
    needed for any frontends.
    """

    description: dict[str, str | int | float]

    @staticmethod
    def from_json_string(json_str: str) -> Optional["TaskBookingLabel"]:
        try:
            return TaskBookingLabel.parse_raw(json_str)
        except pydantic.error_wrappers.ValidationError:
            return None
