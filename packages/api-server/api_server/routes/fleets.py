from typing import List, Optional, cast

from fastapi import Depends, HTTPException, Query
from rx import operators as rxops

from api_server.dependencies import pagination_query, sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import Fleet, FleetState, Pagination, Robot, RobotHealth
from api_server.repositories import RmfRepository, rmf_repo_dep
from api_server.rmf_io import rmf_events

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

    return list(robots.values())


@router.get("/{name}/state", response_model=FleetState)
async def get_fleet_state(name: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)):
    """
    Available in socket.io
    """
    fleet_state = await rmf_repo.get_fleet_state(name)
    if fleet_state is None:
        raise HTTPException(status_code=404)
    return fleet_state


@router.sub("/{name}/state", response_model=FleetState)
async def sub_fleet_state(req: SubscriptionRequest, name: str):
    user = sio_user(req)
    fleet_state = await get_fleet_state(name, RmfRepository(user))
    await req.sio.emit(req.room, fleet_state, req.sid)
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
    robot_health = await rmf_repo.get_robot_health(fleet, robot)
    if robot_health is None:
        raise HTTPException(status_code=404)
    return robot_health


@router.sub("/{fleet}/{robot}/health", response_model=RobotHealth)
async def sub_robot_health(req: SubscriptionRequest, fleet: str, robot: str):
    user = sio_user(req)
    health = await get_robot_health(fleet, robot, RmfRepository(user))
    await req.sio.emit(req.room, health, req.sid)
    return rmf_events.robot_health.pipe(
        rxops.filter(lambda x: cast(RobotHealth, x).id_ == f"{fleet}/{robot}")
    )
