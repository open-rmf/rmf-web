from tortoise.fields import CharField, JSONField
from tortoise.models import Model


class BuildingMap(Model):
    id_ = CharField(255, pk=True, source_field="id")
    data = JSONField()
