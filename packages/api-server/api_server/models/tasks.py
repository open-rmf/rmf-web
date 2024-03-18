from datetime import datetime
from enum import Enum

from pydantic import BaseModel, ConfigDict

from .tortoise_support import TortoiseReverseRelation


class ScheduledTaskSchedule(BaseModel):
    model_config = ConfigDict(from_attributes=True)

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
    start_from: datetime | None = None
    until: datetime | None = None
    period: Period
    at: str | None = None


class ScheduledTask(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int
    task_request: dict
    created_by: str
    schedules: TortoiseReverseRelation[ScheduledTaskSchedule]
    last_ran: datetime | None = None
    except_dates: list[str] | None = None
