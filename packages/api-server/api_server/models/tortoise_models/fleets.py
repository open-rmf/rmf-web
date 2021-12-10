from tortoise.fields import CharField, JSONField, data
from tortoise.models import Model


class FleetState(Model):
    name = CharField(255, pk=True)
    data = JSONField()


class FleetLog(Model):
    name = CharField(255, pk=True)
    data = JSONField()
