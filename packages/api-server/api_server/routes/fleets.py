from typing import List, Tuple, cast

from fastapi import Depends, HTTPException
from rx import operators as rxops

from api_server.authenticator import user_dep
from api_server.dependencies import between_query, sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import rmf_gateway
from api_server.logger import logger
from api_server.models import FleetLog, FleetState, User
from api_server.repositories import FleetRepository, fleet_repo_dep
from api_server.rmf_io import fleet_events

router = FastIORouter(tags=["Fleets"])


@router.get("", response_model=List[FleetState])
async def get_fleets(
    repo: FleetRepository = Depends(fleet_repo_dep),
):
    return await repo.get_all_fleets()


@router.get("/{name}/state", response_model=FleetState)
async def get_fleet_state(name: str, repo: FleetRepository = Depends(fleet_repo_dep)):
    """
    Available in socket.io
    """
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state is None:
        raise HTTPException(status_code=404)
    return fleet_state


@router.sub("/{name}/state", response_model=FleetState)
async def sub_fleet_state(req: SubscriptionRequest, name: str):
    user = sio_user(req)
    repo = FleetRepository(user)
    obs = fleet_events.fleet_states.pipe(
        rxops.filter(lambda x: cast(FleetState, x).name == name)
    )
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state:
        return obs.pipe(rxops.start_with(fleet_state))
    return obs


@router.get("/{name}/log", response_model=FleetLog)
async def get_fleet_log(
    name: str,
    repo: FleetRepository = Depends(fleet_repo_dep),
    between: Tuple[int, int] = Depends(between_query),
):
    """
    Available in socket.io
    """
    fleet_log = await repo.get_fleet_log(name, between)
    if fleet_log is None:
        raise HTTPException(status_code=404)
    return fleet_log


@router.sub("/{name}/log", response_model=FleetLog)
async def sub_fleet_log(_req: SubscriptionRequest, name: str):
    return fleet_events.fleet_logs.pipe(
        rxops.filter(lambda x: cast(FleetLog, x).name == name)
    )


@router.post("/{name}/decommission")
async def decommission_robot(
    name: str,
    robot_name: str,
    request_id: str,
    repo: FleetRepository = Depends(fleet_repo_dep),
    user: User = Depends(user_dep),
):
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state is None:
        raise HTTPException(404, f"Fleet {name} not found")
    if robot_name not in fleet_state.robots:
        raise HTTPException(404, f"Robot {robot_name} not found in fleet {name}")

    logger.info(f"Decommissioning {robot_name} of {name} called by {user.username}")
    rmf_gateway().decommission_robot(
        fleet_name=name,
        robot_name=robot_name,
        request_id=request_id,
    )


@router.post("/{name}/recommission")
async def recommission_robot(
    name: str,
    robot_name: str,
    request_id: str,
    repo: FleetRepository = Depends(fleet_repo_dep),
    user: User = Depends(user_dep),
):
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state is None:
        raise HTTPException(404, f"Fleet {name} not found")
    if robot_name not in fleet_state.robots:
        raise HTTPException(404, f"Robot {robot_name} not found in fleet {name}")

    logger.info(f"Recommissioning {robot_name} of {name} called by ${user.username}")
    rmf_gateway().recommission_robot(
        fleet_name=name, robot_name=robot_name, request_id=request_id
    )
