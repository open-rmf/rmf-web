from tortoise.fields import (
    BooleanField,
    CharField,
    DatetimeField,
    JSONField,
    OneToOneField,
    ReverseRelation,
)
from tortoise.models import Model


class AlertResponse(Model):
    id = CharField(255, pk=True)
    response_time = DatetimeField(null=False, index=True)
    response = CharField(255, null=False, index=True)
    data = JSONField()
    alert_request = OneToOneField(
        "models.AlertRequest", null=False, related_name="alert_response"
    )


class AlertRequest(Model):
    id = CharField(255, pk=True)
    request_time = DatetimeField(null=False, index=True)
    response_expected = BooleanField(null=False, index=True)
    task_id = CharField(255, null=True, index=True)
    data = JSONField()
    alert_response = ReverseRelation["AlertResponse"]
