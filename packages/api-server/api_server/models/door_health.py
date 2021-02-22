from tortoise import fields

from .health_status_mixin import HealthStatusMixin


class DoorHealth(HealthStatusMixin):
    name: fields.CharField(255, pk=True)
