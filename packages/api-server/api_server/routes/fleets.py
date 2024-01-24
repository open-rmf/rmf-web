from typing import List, Tuple

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.dependencies import between_query, sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import FleetLog, FleetState
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
    obs = fleet_events.fleet_states.pipe(rxops.filter(lambda x: x.name == name))
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
    return fleet_events.fleet_logs.pipe(rxops.filter(lambda x: x.name == name))
