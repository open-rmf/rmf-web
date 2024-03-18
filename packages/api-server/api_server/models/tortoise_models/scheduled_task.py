from datetime import datetime
from enum import Enum

import schedule
from schedule import Job
from tortoise.fields import (
    CharEnumField,
    CharField,
    DatetimeField,
    ForeignKeyField,
    ForeignKeyRelation,
    IntField,
    JSONField,
    ReverseRelation,
    SmallIntField,
)
from tortoise.models import Model


class ScheduledTask(Model):
    task_request = JSONField()
    created_by = CharField(255)
    schedules: ReverseRelation["ScheduledTaskSchedule"]
    last_ran = DatetimeField(null=True)
    except_dates = JSONField(null=True)


class ScheduledTaskSchedule(Model):
    """
    The schedules for a scheduled task request.
    A scheduled task may have multiple schedules.
    """

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

    _id = IntField(pk=True, source_field="id")
    scheduled_task: ForeignKeyRelation[ScheduledTask] = ForeignKeyField(
        "models.ScheduledTask", related_name="schedules"
    )
    every = SmallIntField(null=True)
    start_from = DatetimeField(null=True)
    until = DatetimeField(null=True)
    period = CharEnumField(Period)
    at = CharField(255, null=True)

    def get_id(self) -> int:
        return self._id

    def to_job(self) -> Job:
        if self.every is not None:
            job = schedule.every(self.every)
        else:
            job = schedule.every()
        if self.until is not None:
            # schedule uses `datetime.now()`, which is tz naive
            # Assuming self.until is a datetime object with timezone information
            # Convert the timestamp to datetime without changing the timezone
            job = job.until(datetime.utcfromtimestamp(self.until.timestamp()))

        if self.period in (
            ScheduledTaskSchedule.Period.Monday,
            ScheduledTaskSchedule.Period.Tuesday,
            ScheduledTaskSchedule.Period.Wednesday,
            ScheduledTaskSchedule.Period.Thursday,
            ScheduledTaskSchedule.Period.Friday,
            ScheduledTaskSchedule.Period.Saturday,
            ScheduledTaskSchedule.Period.Sunday,
        ):
            job = getattr(job, self.period)
        elif self.period == ScheduledTaskSchedule.Period.Day:
            job = job.days
        elif self.period == ScheduledTaskSchedule.Period.Hour:
            job = job.hours
        elif self.period == ScheduledTaskSchedule.Period.Minute:
            job = job.minutes
        else:
            raise ValueError("invalid period")

        # Hashable value in order to tag the job with a unique identifier
        job.tag(self._id)
        if self.at is not None:
            job = job.at(self.at)

        return job
