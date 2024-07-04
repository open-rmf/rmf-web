import tortoise
from tortoise.fields import CharField, JSONField


class Rio(tortoise.Model):
    id = CharField(max_length=255, pk=True)
    type = CharField(max_length=255, index=True)
    data = JSONField()
