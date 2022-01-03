from typing import List, Optional, cast

from fastapi import Depends, Query
from rx import operators as rxops

from api_server.dependencies import pagination_query, sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import rmf_gateway
from api_server.logger import logger
from api_server.models import Fleet, FleetState, Pagination, Robot, RobotHealth, Task
from api_server.repositories import RmfRepository, rmf_repo_dep
from api_server.rmf_io import rmf_events

from .tasks.utils import get_task_progress

router = FastIORouter(tags=["Fleets"])


@router.get("", response_model=List[Fleet])
async def get_fleets(
    rmf_repo: RmfRepository = Depends(rmf_repo_dep),
    pagination: Pagination = Depends(pagination_query),
    fleet_name: Optional[str] = Query(
        None, description="comma separated list of fleet names"
    ),
):
    return await rmf_repo.query_fleets(pagination, fleet_name=fleet_name)


@router.get("/robots", response_model=List[Robot])
async def get_robots(
    rmf_repo: RmfRepository = Depends(rmf_repo_dep),
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
            logger.warning(
                f'task "{t.task_id}" is assigned to an unknown fleet/robot ({t.fleet_name}/{t.robot_name}'
            )
            continue
        r.tasks.append(
            Task(
                task_id=t.task_id,
                authz_grp=t.authz_grp,
                summary=t,
                progress=get_task_progress(
                    t,
                    rmf_gateway.now(),
                ),
            )
        )

    return list(robots.values())


@router.get("/{name}/state", response_model=FleetState)
async def get_fleet_state(name: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)):
    """
    Available in socket.io
    """
    return await rmf_repo.get_fleet_state(name)


@router.sub("/{name}/state", response_model=FleetState)
async def sub_fleet_state(req: SubscriptionRequest, name: str):
    user = sio_user(req)
    fleet_state = await get_fleet_state(name, RmfRepository(user))
    if fleet_state is not None:
        await req.sio.emit(req.room, fleet_state.dict(), req.sid)
    return rmf_events.fleet_states.pipe(
        rxops.filter(lambda x: cast(FleetState, x).name == name)
    )


@router.get("/{fleet}/{robot}/health", response_model=RobotHealth)
async def get_robot_health(
    fleet: str, robot: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)
):
    """
    Available in socket.io
    """
    return await rmf_repo.get_robot_health(fleet, robot)


@router.sub("/{fleet}/{robot}/health", response_model=RobotHealth)
async def sub_robot_health(req: SubscriptionRequest, fleet: str, robot: str):
    user = sio_user(req)
    health = await get_robot_health(fleet, robot, RmfRepository(user))
    if health is not None:
        await req.sio.emit(req.room, health.dict(), req.sid)
    return rmf_events.robot_health.pipe(
        rxops.filter(lambda x: cast(RobotHealth, x).id_ == f"{fleet}/{robot}")
    )
