from tortoise import Model, fields


class BuildingMap(Model):
    id_ = fields.CharField(255, pk=True, source_field="id")
    data = fields.JSONField()
