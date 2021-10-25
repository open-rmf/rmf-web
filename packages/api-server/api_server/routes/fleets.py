from typing import List, Optional

from fastapi import Depends, Query
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.dependencies import cache_control, pagination_query
from api_server.fast_io import FastIORouter, WatchRequest
from api_server.models import Fleet, FleetState, Pagination, Robot, RobotHealth, Task
from api_server.repositories import RmfRepository

from .tasks.utils import get_task_progress
from .utils import rx_watcher


class FleetsRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Fleets"])
        logger = app.logger

        @self.get(
            "", response_model=List[Fleet], dependencies=[Depends(cache_control())]
        )
        async def get_fleets(
            rmf_repo: RmfRepository = Depends(app.rmf_repo),
            pagination: Pagination = Depends(pagination_query),
            fleet_name: Optional[str] = Query(
                None, description="comma separated list of fleet names"
            ),
        ):
            return await rmf_repo.query_fleets(pagination, fleet_name=fleet_name)

        @self.get(
            "/robots",
            response_model=List[Robot],
            dependencies=[Depends(cache_control())],
        )
        async def get_robots(
            rmf_repo: RmfRepository = Depends(app.rmf_repo),
            pagination: Pagination = Depends(pagination_query),
            fleet_name: Optional[str] = Query(
                None, description="comma separated list of fleet names"
            ),
            robot_name: Optional[str] = Query(
                None, description="comma separated list of robot names"
            ),
        ):
            robots = {
                f"{r.fleet}/{r.name}": r
                for r in await rmf_repo.query_robots(
                    pagination,
                    fleet_name=fleet_name,
                    robot_name=robot_name,
                )
            }

            filter_states = [
                "active",
                "pending",
                "queued",
            ]

            tasks_pagination = Pagination(limit=100, offset=0, order_by="start_time")
            tasks = await rmf_repo.query_task_summaries(
                tasks_pagination,
                fleet_name=fleet_name,
                robot_name=robot_name,
                state=",".join(filter_states),
            )

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
                r.tasks.append(
                    Task(
                        task_id=t.task_id,
                        authz_grp=t.authz_grp,
                        summary=t,
                        progress=get_task_progress(
                            t,
                            app.rmf_gateway().now(),
                        ),
                    )
                )

            return list(robots.values())

        @self.get("/{name}/state", response_model=FleetState)
        async def get_fleet_state(
            name: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_fleet_state(name)

        @self.watch("/{name}/state")
        async def watch_fleet_state(req: WatchRequest, name: str):
            fleet_state = await get_fleet_state(name, RmfRepository(req.user))
            await req.emit(fleet_state.dict())
            rx_watcher(
                req,
                app.rmf_events().fleet_states.pipe(
                    rxops.filter(lambda x: x.name == name),
                    rxops.map(lambda x: x.dict()),
                ),
            )

        @self.get("/{fleet}/{robot}/health", response_model=RobotHealth)
        async def get_robot_health(
            fleet: str, robot: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_robot_health(fleet, robot)

        @self.watch("/{fleet}/{robot}/health")
        async def watch_robot_health(req: WatchRequest, fleet: str, robot: str):
            health = await get_robot_health(fleet, robot, RmfRepository(req.user))
            await req.emit(health.dict())
            rx_watcher(
                req,
                app.rmf_events().robot_health.pipe(
                    rxops.filter(lambda x: x.id_ == f"{fleet}/{robot}"),
                    rxops.map(lambda x: x.dict()),
                ),
            )
