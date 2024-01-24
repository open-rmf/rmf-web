from typing import List

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.dependencies import sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import rmf_gateway
from api_server.models import Lift, LiftHealth, LiftRequest, LiftState
from api_server.repositories import RmfRepository, rmf_repo_dep
from api_server.rmf_io import rmf_events

router = FastIORouter(tags=["Lifts"])


@router.get("", response_model=List[Lift])
async def get_lifts(rmf_repo: RmfRepository = Depends(rmf_repo_dep)):
    return await rmf_repo.get_lifts()


@router.get("/{lift_name}/state", response_model=LiftState)
async def get_lift_state(
    lift_name: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)
):
    """
    Available in socket.io
    """
    lift_state = await rmf_repo.get_lift_state(lift_name)
    if lift_state is None:
        raise HTTPException(status_code=404)
    return lift_state


@router.sub("/{lift_name}/state", response_model=LiftState)
async def sub_lift_state(req: SubscriptionRequest, lift_name: str):
    user = sio_user(req)
    obs = rmf_events.lift_states.pipe(rxops.filter(lambda x: x.lift_name == lift_name))
    lift_state = await get_lift_state(lift_name, RmfRepository(user))
    if lift_state:
        return obs.pipe(rxops.start_with(lift_state))
    return obs


@router.get("/{lift_name}/health", response_model=LiftHealth)
async def get_lift_health(
    lift_name: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)
):
    """
    Available in socket.io
    """
    lift_health = await rmf_repo.get_lift_health(lift_name)
    if lift_health is None:
        raise HTTPException(status_code=404)
    return lift_health


@router.sub("/{lift_name}/health", response_model=LiftHealth)
async def sub_lift_health(req: SubscriptionRequest, lift_name: str):
    user = sio_user(req)
    obs = rmf_events.lift_health.pipe(rxops.filter(lambda x: x.id_ == lift_name))
    health = await get_lift_health(lift_name, RmfRepository(user))
    if health:
        return obs.pipe(rxops.start_with(health))
    return obs


@router.post("/{lift_name}/request")
def _post_lift_request(
    lift_name: str,
    lift_request: LiftRequest,
):
    rmf_gateway().request_lift(
        lift_name,
        lift_request.destination,
        lift_request.request_type,
        lift_request.door_mode,
    )
