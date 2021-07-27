from tortoise import Model, fields

from api_server.models.fleets import FleetState as PydanticFleetState
from api_server.models.fleets import RobotState as PydanticRobotState
from api_server.models.tortoise_models.json_mixin import JsonMixin


class FleetState(Model, JsonMixin):
    @staticmethod
    async def save_pydantic(fleet_state: PydanticFleetState):
        await FleetState.update_or_create(
            {
                "data": fleet_state.dict(),
            },
            id_=fleet_state.name,
        )

    def to_pydantic(self):
        return PydanticFleetState(**self.data)


class RobotState(Model):
    fleet_name = fields.CharField(255)
    robot_name = fields.CharField(255)
    data = fields.JSONField()

    class Meta:
        unique_together = ("fleet_name", "robot_name")

    @staticmethod
    async def save_pydantic(fleet_name: str, robot_state: PydanticRobotState):
        await RobotState.update_or_create(
            {
                "data": robot_state.dict(),
            },
            fleet_name=fleet_name,
            robot_name=robot_state.name,
        )

    def to_pydantic(self):
        return PydanticRobotState(**self.data)
