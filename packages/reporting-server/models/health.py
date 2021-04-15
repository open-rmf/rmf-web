from enum import Enum

from tortoise import fields, models


class HealthStatusEmun(Enum):
    HEALTHY = "Healthy"
    UNHEALTHY = "Unhealthy"
    DEAD = "Dead"


class HealthStatus(models.Model):
    device = fields.TextField()
    actor_id = fields.CharField(max_length=25, null=True)
    health_status = fields.CharField(max_length=25, null=True)
    health_message = fields.TextField(null=True)
    payload = fields.JSONField()
