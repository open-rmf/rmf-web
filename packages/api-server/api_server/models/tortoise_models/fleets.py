from tortoise.fields import CharField, JSONField
from tortoise.models import Model


class FleetState(Model):
    name = CharField(255, pk=True)
    data = JSONField()
