from tortoise.fields import (
    CharField,
    DatetimeField,
    ForeignKeyField,
    ForeignKeyRelation,
    JSONField,
    ReverseRelation,
)
from tortoise.models import Model

from .log import LogMixin


class TaskState(Model):
    id_ = CharField(255, pk=True, source_field="id")
    data = JSONField()
    category = CharField(255, null=True, index=True)
    assigned_to = CharField(255, null=True, index=True)
    unix_millis_start_time = DatetimeField(null=True, index=True)
    unix_millis_finish_time = DatetimeField(null=True, index=True)
    status = CharField(255, null=True, index=True)


class TaskEventLog(Model):
    task_id = CharField(255, pk=True)
    log: ReverseRelation["TaskEventLogLog"]
    phases: ReverseRelation["TaskEventLogPhases"]


class TaskEventLogLog(Model, LogMixin):
    task: ForeignKeyRelation[TaskEventLog] = ForeignKeyField("models.TaskEventLog", related_name="log")  # type: ignore

    class Meta:
        unique_together = ("task", "seq")


class TaskEventLogPhases(Model):
    task: ForeignKeyRelation[TaskEventLog] = ForeignKeyField("models.TaskEventLog", related_name="phases")  # type: ignore
    phase = CharField(255)
    log: ReverseRelation["TaskEventLogPhasesLog"]
    events: ReverseRelation["TaskEventLogPhasesEvents"]


class TaskEventLogPhasesLog(Model, LogMixin):
    phase: ForeignKeyRelation[TaskEventLogPhases] = ForeignKeyField("models.TaskEventLogPhases", related_name="log")  # type: ignore

    class Meta:
        unique_together = ("id", "seq")


class TaskEventLogPhasesEvents(Model):
    phase: ForeignKeyRelation[TaskEventLogPhases] = ForeignKeyField("models.TaskEventLogPhases", related_name="events")  # type: ignore
    event = CharField(255)
    log: ReverseRelation["TaskEventLogPhasesEventsLog"]


class TaskEventLogPhasesEventsLog(Model, LogMixin):
    event: ForeignKeyRelation[TaskEventLogPhasesEvents] = ForeignKeyField("models.TaskEventLogPhasesEvents", related_name="log")  # type: ignore

    class Meta:
        unique_together = ("id", "seq")
