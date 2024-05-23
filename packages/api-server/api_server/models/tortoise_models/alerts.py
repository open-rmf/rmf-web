from tortoise.fields import CharField, ForeignKeyField, JSONField, ReverseRelation
from tortoise.models import Model


class AlertResponse(Model):
    id = CharField(255, pk=True)
    alert_request = ForeignKeyField(
        "models.AlertRequest", null=True, related_name="alert_response"
    )
    data = JSONField()


class AlertRequest(Model):
    id = CharField(255, pk=True)
    data = JSONField()
    task_id = CharField(255, null=True, index=True)
    alert_response = ReverseRelation["FleetAlertResponse"]


# how to let backend know that the robot is ready for handling
# how does the backend save this information such that the smart cart API server can query it
# new location and destination model
# when an alert comes in with task id, and with alert parameters
# reached: xx, we update new model for task, so SCAS queries and can update

# how do we change destinations?
