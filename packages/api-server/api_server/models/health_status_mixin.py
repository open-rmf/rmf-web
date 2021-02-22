from enum import Enum

from tortoise import fields


class HealthStatus(Enum):
    HEALTHY = "Healthy"
    UNHEALTHY = "Unhealthy"
    DEAD = "Dead"


class HealthStatusMixin:
    health_status: fields.CharEnumField(HealthStatus, max_length=255, null=True)
