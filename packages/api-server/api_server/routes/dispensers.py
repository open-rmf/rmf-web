from typing import List, cast

from fastapi import Depends, HTTPException
from rx import operators as rxops
from rx.subject.replaysubject import ReplaySubject

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
    dispenser_state = await get_dispenser_state(guid, RmfRepository(user))
    sub = ReplaySubject(1)
    if dispenser_state:
        sub.on_next(dispenser_state)
    rmf_events.dispenser_states.pipe(
        rxops.filter(lambda x: cast(DispenserState, x).guid == guid)
    ).subscribe(sub)
    return sub


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
    health = await get_dispenser_health(guid, RmfRepository(user))
    await req.sio.emit(req.room, health, req.sid)
    return rmf_events.dispenser_health.pipe(
        rxops.filter(lambda x: cast(DispenserHealth, x).id_ == guid)
    )
