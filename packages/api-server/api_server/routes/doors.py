from typing import List

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

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
    obs = rmf_events.door_states.pipe(rxops.filter(lambda x: x.door_name == door_name))
    door_state = await get_door_state(door_name, RmfRepository(user))
    if door_state:
        return obs.pipe(rxops.start_with(door_state))
    return obs


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
    obs = rmf_events.door_health.pipe(rxops.filter(lambda x: x.id_ == door_name))
    health = await get_door_health(door_name, RmfRepository(user))
    if health:
        return obs.pipe(rxops.start_with(health))
    return obs


@router.post("/{door_name}/request")
def post_door_request(
    door_name: str,
    door_request: DoorRequest,
):
    rmf_gateway().request_door(door_name, door_request.mode)
