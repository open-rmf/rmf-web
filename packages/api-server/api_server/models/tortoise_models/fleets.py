from tortoise.fields import (
    CharField,
    ForeignKeyField,
    ForeignKeyRelation,
    JSONField,
    ReverseRelation,
)
from tortoise.models import Model

from .log import LogMixin


class FleetState(Model):
    name = CharField(255, pk=True)
    data = JSONField()


class FleetLog(Model):
    name = CharField(255, pk=True)
    log: ReverseRelation["FleetLogLog"]
    robots: ReverseRelation["FleetLogRobots"]


class FleetLogLog(Model, LogMixin):
    fleet: ForeignKeyRelation[FleetLog] = ForeignKeyField(
        "models.FleetLog", related_name="log"
    )

    class Meta:  # type: ignore
        unique_together = ("fleet", "seq")


class FleetLogRobots(Model):
    fleet: ForeignKeyRelation[FleetLog] = ForeignKeyField(
        "models.FleetLog", related_name="robots"
    )
    name = CharField(255)
    log: ReverseRelation["FleetLogRobotsLog"]


class FleetLogRobotsLog(Model, LogMixin):
    robot: ForeignKeyRelation[FleetLogRobots] = ForeignKeyField(
        "models.FleetLogRobots", related_name="log"
    )

    class Meta:  # type: ignore
        unique_together = ("id", "seq")
