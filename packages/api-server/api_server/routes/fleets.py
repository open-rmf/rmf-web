from typing import List, Optional, cast

from fastapi import Depends, HTTPException, Query
from rx import operators as rxops
from rx.subject.replaysubject import ReplaySubject

from api_server.dependencies import pagination_query, sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import FleetLog, FleetState, Pagination
from api_server.models.tortoise_models import FleetState as DbFleetState
from api_server.repositories import FleetRepository, fleet_repo_dep
from api_server.rmf_io import fleet_events

router = FastIORouter(tags=["Fleets"])

router = FastIORouter(tags=["Fleets"])


@router.get("", response_model=List[FleetState])
async def query_fleets(
    repo: FleetRepository = Depends(fleet_repo_dep),
    pagination: Pagination = Depends(pagination_query),
    fleet_name: Optional[str] = Query(
        None, description="comma separated list of fleet names"
    ),
):
    filters = {}
    if fleet_name is not None:
        filters["name__in"] = fleet_name.split(",")
    return await repo.query_fleet_states(DbFleetState.filter(**filters), pagination)


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
    fleet_state = await get_fleet_state(name, FleetRepository(user))
    sub = ReplaySubject(1)
    if fleet_state:
        sub.on_next(fleet_state)
    fleet_events.fleet_states.subscribe(sub)
    return sub


@router.get("/{name}/log", response_model=FleetLog)
async def get_fleet_log(name: str, repo: FleetRepository = Depends(fleet_repo_dep)):
    """
    Available in socket.io
    """
    fleet_log = await repo.get_fleet_log(name)
    if fleet_log is None:
        raise HTTPException(status_code=404)
    return fleet_log


@router.sub("/{name}/log", response_model=FleetLog)
async def sub_fleet_log(req: SubscriptionRequest, name: str):
    user = sio_user(req)
    fleet_log = await get_fleet_log(name, FleetRepository(user))
    sub = ReplaySubject(1)
    if fleet_log:
        sub.on_next(fleet_log)
    fleet_events.fleet_logs.subscribe(sub)
    return sub
