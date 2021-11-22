from tortoise.fields.data import CharField
from tortoise.models import Model

from .health_status_mixin import HealthStatusMixin


class BasicHealthModel(Model, HealthStatusMixin):
    id_ = CharField(255, pk=True, source_field="id")


class DoorHealth(BasicHealthModel):
    pass


class LiftHealth(BasicHealthModel):
    pass


class DispenserHealth(BasicHealthModel):
    pass


class IngestorHealth(BasicHealthModel):
    pass


class RobotHealth(BasicHealthModel):
    pass
