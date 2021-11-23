from tortoise.fields.data import CharField, TextField


class HealthStatusMixin:
    health_status = CharField(max_length=255, null=True)
    health_message = TextField(null=True)
