from typing import List, cast

from fastapi import Depends, HTTPException
from rx import operators as rxops
from rx.subject.replaysubject import ReplaySubject

from api_server.dependencies import sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import rmf_gateway
from api_server.models import Door, DoorHealth, DoorRequest, DoorState
from api_server.repositories import RmfRepository, rmf_repo_dep
from api_server.rmf_io import rmf_events

router = FastIORouter(tags=["Doors"])


@router.get("", response_model=List[Door])
async def get_doors(rmf_repo: RmfRepository = Depends(rmf_repo_dep)):
    return await rmf_repo.get_doors()


@router.get("/{door_name}/state", response_model=DoorState)
async def get_door_state(
    door_name: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)
):
    """
    Available in socket.io
    """
    door_state = await rmf_repo.get_door_state(door_name)
    if door_state is None:
        raise HTTPException(status_code=404)
    return door_state


@router.sub("/{door_name}/state", response_model=DoorState)
async def sub_door_state(req: SubscriptionRequest, door_name: str):
    user = sio_user(req)
    door_state = await get_door_state(door_name, RmfRepository(user))
    sub = ReplaySubject(1)
    if door_name:
        sub.on_next(door_state)
    rmf_events.door_states.subscribe(sub)
    return sub


@router.get("/{door_name}/health", response_model=DoorHealth)
async def get_door_health(
    door_name: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)
):
    """
    Available in socket.io
    """
    door_health = await rmf_repo.get_door_health(door_name)
    if door_health is None:
        raise HTTPException(status_code=404)
    return door_health


@router.sub("/{door_name}/health", response_model=DoorHealth)
async def sub_door_health(req: SubscriptionRequest, door_name: str):
    user = sio_user(req)
    health = await get_door_health(door_name, RmfRepository(user))
    await req.sio.emit(req.room, health, req.sid)
    return rmf_events.door_health.pipe(
        rxops.filter(lambda x: cast(DoorHealth, x).id_ == door_name)
    )


@router.post("/{door_name}/request")
def post_door_request(
    door_name: str,
    door_request: DoorRequest,
):
    rmf_gateway.request_door(door_name, door_request.mode)
