from typing import List, Optional

from fastapi import Depends, Query
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.dependencies import AddPaginationQuery, pagination_query
from api_server.fast_io import FastIORouter, WatchRequest
from api_server.models import Fleet, FleetState, RobotHealth, Task
from api_server.models import tortoise_models as ttm
from api_server.models.fleets import Robot
from api_server.models.tasks import TaskStateEnum
from api_server.routes.tasks.utils import get_task_progress
from api_server.routes.utils import rx_watcher


class FleetsRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Fleets"])
        logger = app.logger

        @self.get("", response_model=List[Fleet])
        async def get_fleets(
            add_pagination: AddPaginationQuery[ttm.FleetState] = Depends(
                pagination_query({"fleet_name": "id_"})
            ),
            fleet_name: Optional[str] = Query(
                None, description="comma separated list of fleet names"
            ),
        ):
            filter_params = {}
            if fleet_name is not None:
                filter_params["id___in"] = fleet_name.split(",")
            states = await add_pagination(ttm.FleetState.filter(**filter_params))
            states = [s.to_pydantic() for s in states]
            return [Fleet(name=s.name, state=s) for s in states]

        @self.get("/robots", response_model=List[Robot])
        async def get_robots(
            add_pagination: AddPaginationQuery[ttm.RobotState] = Depends(
                pagination_query()
            ),
            fleet_name: Optional[str] = Query(
                None, description="comma separated list of fleet names"
            ),
            robot_name: Optional[str] = Query(
                None, description="comma separated list of robot names"
            ),
        ):
            filter_params = {}
            if fleet_name is not None:
                filter_params["fleet_name__in"] = fleet_name.split(",")
            if robot_name is not None:
                filter_params["robot_name__in"] = robot_name.split(",")

            robot_states = await add_pagination(ttm.RobotState.filter(**filter_params))
            robots = {
                f"{q.fleet_name}/{q.robot_name}": Robot(
                    fleet=q.fleet_name, name=q.robot_name, state=q.data
                )
                for q in robot_states
            }

            filter_states = [
                TaskStateEnum.ACTIVE.value,
                TaskStateEnum.PENDING.value,
                TaskStateEnum.QUEUED.value,
            ]
            tasks = await ttm.TaskSummary.filter(
                state__in=filter_states,
                **filter_params,
            ).order_by("start_time")
            for t in tasks:
                r = robots.get(f"{t.fleet_name}/{t.robot_name}", None)
                # This should only happen under very rare scenarios, when there are
                # multiple fleets with the same robot name and there are active tasks
                # assigned to those robots and the robot states are not synced to the
                # tasks summaries.
                if r is None:
                    logger.warn(
                        f'task "{t.id_}" is assigned to an unknown fleet/robot ({t.fleet_name}/{t.robot_name}'
                    )
                ts = t.to_pydantic()
                r.tasks.append(
                    Task(
                        task_id=ts.task_id,
                        authz_grp=t.authz_grp,
                        summary=ts,
                        progress=get_task_progress(
                            t.to_pydantic(),
                            app.rmf_gateway().get_clock().now().to_msg(),
                        ),
                    )
                )

            return list(robots.values())

        @self.get("/{name}/state", response_model=FleetState)
        async def get_fleet_state(name: str):
            """
            Available in socket.io
            """
            return await app.rmf_repo.get_fleet_state(name)

        @self.watch("/{name}/state")
        async def watch_fleet_state(req: WatchRequest, name: str):
            await req.emit(await get_fleet_state(name))
            rx_watcher(
                req,
                app.rmf_events().fleet_states.pipe(
                    rxops.filter(lambda x: x.name == name)
                ),
            )

        @self.get("/{fleet}/{robot}/health", response_model=RobotHealth)
        async def get_robot_health(fleet: str, robot: str):
            """
            Available in socket.io
            """
            return await app.rmf_repo.get_robot_health(fleet, robot)

        @self.watch("/{fleet}/{robot}/health")
        async def watch_robot_health(req: WatchRequest, fleet: str, robot: str):
            await req.emit(await get_robot_health(fleet, robot))
            rx_watcher(
                req,
                app.rmf_events().robot_health.pipe(
                    rxops.filter(lambda x: x.id_ == f"{fleet}/{robot}")
                ),
            )
