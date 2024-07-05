from datetime import date, datetime
from enum import StrEnum

from pydantic import BaseModel, ConfigDict, Field

from .rmf_api.task_request import TaskRequest
from .tortoise_support import TortoiseReverseRelation


class ScheduledTaskSchedule(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    class Period(StrEnum):
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
    start_from: datetime | None = None
    until: datetime | None = None
    period: Period
    at: str


class ScheduledTask(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int
    task_request: TaskRequest
    created_by: str
    schedules: TortoiseReverseRelation[ScheduledTaskSchedule]
    last_ran: datetime | None = None
    except_dates: list[date] = Field(default=[])
