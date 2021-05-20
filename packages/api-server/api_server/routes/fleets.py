from typing import List

from api_server.models.fleets import Robot

from ..fast_io import FastIORouter
from ..models import Fleet, FleetState, RobotHealth
from ..models import tortoise_models as ttm
from ..repositories import RmfRepository
from ..rmf_io import RmfEvents


class FleetsRouter(FastIORouter):
    def __init__(
        self,
        rmf_events: RmfEvents,
        rmf_repo: RmfRepository,
    ):
        super().__init__(tags=["Fleets"])

        @self.get("", response_model=List[Fleet])
        async def get_fleets():
            return await rmf_repo.query_fleets()

        @self.get("/robots", response_model=List[Robot])
        async def get_robots():
            fleet_states = [f.to_pydantic() for f in await ttm.FleetState.all()]
            robots = []
            for fleet_state in fleet_states:
                robots.extend(
                    Robot(fleet=fleet_state.name, name=r.name)
                    for r in fleet_state.robots
                )
            return robots

        @self.watch("/{name}/state", rmf_events.fleet_states, response_model=FleetState)
        def get_fleet_state(fleet_state: FleetState):
            return {"name": fleet_state.name}, fleet_state

        @self.watch(
            "/{fleet}/{robot}/health",
            rmf_events.robot_health,
            response_model=RobotHealth,
        )
        def get_robot_health(robot_health: RobotHealth):
            fleet, robot = robot_health.id_.split("/")
            return {"fleet": fleet, "robot": robot}, robot_health
