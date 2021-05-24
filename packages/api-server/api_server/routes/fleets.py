from typing import List, Optional

from fastapi import Depends, Query

from api_server.models.fleets import Robot
from api_server.models.pagination import Pagination

from ..dependencies import WithBaseQuery, base_query_params
from ..fast_io import FastIORouter
from ..models import Fleet, FleetState, RobotHealth
from ..models import tortoise_models as ttm
from ..rmf_io import RmfEvents


class FleetsRouter(FastIORouter):
    def __init__(
        self,
        rmf_events: RmfEvents,
    ):
        super().__init__(tags=["Fleets"])

        class GetFleetsResponse(Pagination.response_model(Fleet)):
            pass

        @self.get("", response_model=GetFleetsResponse)
        async def get_fleets(
            with_base_query: WithBaseQuery[ttm.FleetState] = Depends(
                base_query_params({"fleet_name": "id_"})
            ),
            fleet_name: Optional[str] = Query(
                None, description="comma separated list of fleet names"
            ),
        ):
            filter_params = {}
            if fleet_name is not None:
                filter_params["id___in"] = fleet_name.split(",")
            states = await with_base_query(ttm.FleetState.filter(**filter_params))
            results: Pagination[Fleet] = Pagination(
                total_count=states.total_count,
                items=[Fleet(name=s.id_, state=s.data) for s in states.items],
            )
            return results

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
