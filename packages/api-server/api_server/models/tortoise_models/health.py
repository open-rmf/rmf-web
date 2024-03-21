from tortoise.fields import CharField, TextField
from tortoise.models import Model


class BasicHealthModel(Model):
    id_ = CharField(255, pk=True, source_field="id")
    health_status = CharField(max_length=255, null=True)
    health_message = TextField(null=True)

    class Meta:  # type: ignore
        abstract = True


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
