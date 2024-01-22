from pydantic import BaseModel


class HealthStatus:
    HEALTHY = "Healthy"
    UNHEALTHY = "Unhealthy"
    DEAD = "Dead"


class BaseBasicHealthModel(BaseModel):
    id_: str
    health_status: str
    health_message: str | None
