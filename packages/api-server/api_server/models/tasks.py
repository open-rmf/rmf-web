from datetime import date, datetime
from enum import Enum

from pydantic import Field

from .base import PydanticModel
from .rmf_api.task_request import TaskRequest
from .tortoise_support import TortoiseReverseRelation


class ScheduledTaskSchedule(PydanticModel):
    class Period(str, Enum):
        Monday = "monday"
        Tuesday = "tuesday"
        Wednesday = "wednesday"
        Thursday = "thursday"
        Friday = "friday"
        Saturday = "saturday"
        Sunday = "sunday"
        Day = "day"
        Hour = "hour"
        Minute = "minute"

    every: int | None = None
    period: Period
    at: str


class ScheduledTask(PydanticModel):
    id: int
    task_request: TaskRequest
    created_by: str
    schedules: TortoiseReverseRelation[ScheduledTaskSchedule]
    last_ran: datetime | None = None
    start_from: datetime | None = None
    until: datetime | None = None
    except_dates: list[date] = Field(default=[])
