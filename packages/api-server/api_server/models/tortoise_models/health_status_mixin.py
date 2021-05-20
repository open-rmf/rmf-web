from tortoise import fields


class HealthStatusMixin:
    health_status = fields.CharField(max_length=255, null=True)
    health_message = fields.TextField(null=True)
