from typing import Optional

from pydantic import BaseModel


class HealthStatus:
    HEALTHY = "Healthy"
    UNHEALTHY = "Unhealthy"
    DEAD = "Dead"


class BasicHealth(BaseModel):
    id_: str
    health_status: str
    health_message: Optional[str]
