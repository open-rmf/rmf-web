from typing import List

from ..fast_io import FastIORouter
from ..models import Fleet, FleetState, RobotHealth
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

        @self.watch("/{name}/state", rmf_events.fleet_states, response_model=FleetState)
        def get_fleet_state(fleet_state: FleetState):
            return {"name": fleet_state.name}, fleet_state

        @self.watch(
            "/{fleet}/{robot}/health",
            rmf_events.robot_health,
            response_model=RobotHealth.PydanticModel,  # pylint: disable=no-member
        )
        def get_robot_health(robot_health: RobotHealth):
            fleet, robot = robot_health.id_.split("/")
            return {"fleet": fleet, "robot": robot}, robot_health.get_pydantic()
