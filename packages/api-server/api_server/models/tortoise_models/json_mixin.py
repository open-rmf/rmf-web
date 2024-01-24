from tortoise.fields import CharField, JSONField


class JsonMixin:
    id_ = CharField(255, pk=True, source_field="id")
    data: dict = JSONField()  # type: ignore
