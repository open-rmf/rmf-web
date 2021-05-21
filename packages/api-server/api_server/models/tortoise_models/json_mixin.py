from tortoise import fields


class JsonMixin:
    id_ = fields.CharField(255, pk=True, source_field="id")
    data = fields.JSONField()
