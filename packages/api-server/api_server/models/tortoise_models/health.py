from tortoise import Model, fields

from .health_status_mixin import HealthStatusMixin
from .pydantic_mixins import with_pydantic


@with_pydantic
class BasicHealthModel(Model, HealthStatusMixin):
    id_ = fields.CharField(255, pk=True, source_field="id")

    def to_dict(self):
        result = super().to_dict()
        result.update({"id": self.id_})
        return result


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
