from tortoise.fields import CharField, JSONField
from tortoise.models import Model


class Rio(Model):
    id = CharField(max_length=255, pk=True)
    type = CharField(max_length=255, index=True)
    data = JSONField()
