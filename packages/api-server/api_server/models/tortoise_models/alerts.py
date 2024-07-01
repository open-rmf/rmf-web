from tortoise.fields import (
    BooleanField,
    CharField,
    JSONField,
    OneToOneField,
    ReverseRelation,
)
from tortoise.models import Model


class AlertResponse(Model):
    id = CharField(255, pk=True)
    alert_request = OneToOneField(
        "models.AlertRequest", null=False, related_name="alert_response"
    )
    data = JSONField()


class AlertRequest(Model):
    id = CharField(255, pk=True)
    data = JSONField()
    response_expected = BooleanField(null=False, index=True)
    task_id = CharField(255, null=True, index=True)
    alert_response = ReverseRelation["AlertResponse"]
