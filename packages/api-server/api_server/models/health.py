from . import tortoise_models as ttm

BasicHealth = ttm.BasicHealthModel


class HealthStatus:
    HEALTHY = "Healthy"
    UNHEALTHY = "Unhealthy"
    DEAD = "Dead"
