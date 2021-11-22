from tortoise.fields.data import CharField, JSONField
from tortoise.models import Model

from .json_mixin import JsonMixin


class FleetState(Model, JsonMixin):
    pass


class RobotState(Model):
    fleet_name = CharField(255)
    robot_name = CharField(255)
    data = JSONField()

    class Meta:
        unique_together = ("fleet_name", "robot_name")
