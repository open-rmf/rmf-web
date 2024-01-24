from enum import Enum

from pydantic import BaseModel


class HealthStatus(str, Enum):
    HEALTHY = "Healthy"
    UNHEALTHY = "Unhealthy"
    DEAD = "Dead"


class BasicHealth(BaseModel):
    id_: str
    health_status: HealthStatus
    health_message: str | None
