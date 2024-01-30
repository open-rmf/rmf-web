from typing import List

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.dependencies import sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import Dispenser, DispenserHealth, DispenserState
from api_server.repositories import RmfRepository, rmf_repo_dep
from api_server.rmf_io import rmf_events

router = FastIORouter(tags=["Dispensers"])


@router.get("", response_model=List[Dispenser])
async def get_dispensers(rmf_repo: RmfRepository = Depends(rmf_repo_dep)):
    return await rmf_repo.get_dispensers()


@router.get("/{guid}/state", response_model=DispenserState)
async def get_dispenser_state(
    guid: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)
):
    """
    Available in socket.io
    """
    dispenser_state = await rmf_repo.get_dispenser_state(guid)
    if dispenser_state is None:
        raise HTTPException(status_code=404)
    return dispenser_state


@router.sub("/{guid}/state", response_model=DispenserState)
async def sub_dispenser_state(req: SubscriptionRequest, guid: str):
    user = sio_user(req)
    obs = rmf_events.dispenser_states.pipe(rxops.filter(lambda x: x.guid == guid))
    dispenser_state = await get_dispenser_state(guid, RmfRepository(user))
    if dispenser_state:
        return obs.pipe(rxops.start_with(dispenser_state))
    return obs


@router.get("/{guid}/health", response_model=DispenserHealth)
async def get_dispenser_health(
    guid: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)
):
    """
    Available in socket.io
    """
    dispenser_health = await rmf_repo.get_dispenser_health(guid)
    if dispenser_health is None:
        raise HTTPException(status_code=404)
    return dispenser_health


@router.sub("/{guid}/health", response_model=DispenserHealth)
async def sub_dispenser_health(req: SubscriptionRequest, guid: str):
    user = sio_user(req)
    obs = rmf_events.dispenser_health.pipe(rxops.filter(lambda x: x.id_ == guid))
    health = await get_dispenser_health(guid, RmfRepository(user))
    if health:
        return obs.pipe(rxops.start_with(health))
    return obs
