from typing import List

from pydantic import BaseModel

from api_server.models import tortoise_models as ttm
from api_server.models.health import basic_health_model
from api_server.models.ros_pydantic import rmf_fleet_msgs
from api_server.models.tasks import Task

RobotMode = rmf_fleet_msgs.RobotMode
RobotHealth = basic_health_model(ttm.RobotHealth)
Location = rmf_fleet_msgs.Location


class FleetState(rmf_fleet_msgs.FleetState):
    @staticmethod
    def from_tortoise(tortoise: ttm.FleetState) -> "FleetState":
        return FleetState(**tortoise.data)

    async def save(self) -> None:
        await ttm.FleetState.update_or_create({"data": self.dict()}, id_=self.name)


class Fleet(BaseModel):
    name: str
    state: FleetState


class RobotState(rmf_fleet_msgs.RobotState):
    @staticmethod
    def from_tortoise(tortoise: ttm.RobotState) -> "RobotState":
        return RobotState(**tortoise.data)

    async def save(self, fleet_name: str) -> None:
        await ttm.RobotState.update_or_create(
            {"data": self.dict()}, fleet_name=fleet_name, robot_name=self.name
        )


class Robot(BaseModel):
    fleet: str
    name: str
    state: RobotState
    tasks: List[Task] = []
