from tortoise import Model, fields

from api_server.models.tortoise_models.json_mixin import JsonMixin


class FleetState(Model, JsonMixin):
    pass


class RobotState(Model):
    fleet_name = fields.CharField(255)
    robot_name = fields.CharField(255)
    data = fields.JSONField()

    class Meta:
        unique_together = ("fleet_name", "robot_name")
