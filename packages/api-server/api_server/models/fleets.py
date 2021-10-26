from typing import List

from pydantic import BaseModel

from . import tortoise_models as ttm
from .health import basic_health_model
from .ros_pydantic import rmf_fleet_msgs
from .tasks import Task

RobotMode = rmf_fleet_msgs.RobotMode
RobotHealth = basic_health_model(ttm.RobotHealth)
Location = rmf_fleet_msgs.Location


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


class FleetState(rmf_fleet_msgs.FleetState):
    robots: List[RobotState]

    def __init__(
        self,
        name: str = "",  # string
        robots: List[RobotState] = None,  # rmf_fleet_msgs/RobotState
        **kwargs,
    ):
        super().__init__(
            name=name,
            robots=robots or [],
            **kwargs,
        )

    @staticmethod
    def from_tortoise(tortoise: ttm.FleetState) -> "FleetState":
        return FleetState(**tortoise.data)

    async def save(self) -> None:
        await ttm.FleetState.update_or_create({"data": self.dict()}, id_=self.name)


class Fleet(BaseModel):
    name: str
    state: FleetState
