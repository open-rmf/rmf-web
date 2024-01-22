from typing import Any, cast

from tortoise.fields import (
    CharField,
    Field,
    ForeignKeyField,
    ForeignKeyRelation,
    JSONField,
    ReverseRelation,
)
from tortoise.models import Model

from .log import LogMixin


class FleetState(Model):
    name = CharField(255, pk=True)
    data = cast(Field[dict[str, Any]], JSONField())


class FleetLog(Model):
    name = CharField(255, pk=True)
    log: ReverseRelation["FleetLogLog"]
    robots: ReverseRelation["FleetLogRobots"]


class FleetLogLog(Model, LogMixin):
    fleet: ForeignKeyRelation[FleetLog] = ForeignKeyField("models.FleetLog", related_name="log")  # type: ignore

    class Meta:
        unique_together = ("fleet", "seq")


class FleetLogRobots(Model):
    fleet: ForeignKeyRelation[FleetLog] = ForeignKeyField("models.FleetLog", related_name="robots")  # type: ignore
    name = CharField(255)
    log: ReverseRelation["FleetLogRobotsLog"]


class FleetLogRobotsLog(Model, LogMixin):
    robot: ForeignKeyRelation[FleetLogRobots] = ForeignKeyField("models.FleetLogRobots", related_name="log")  # type: ignore

    class Meta:
        unique_together = ("id", "seq")
