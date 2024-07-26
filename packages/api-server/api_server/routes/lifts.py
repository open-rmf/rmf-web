from typing import Annotated

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import RmfGateway, get_rmf_gateway
from api_server.models import Lift, LiftRequest, LiftState
from api_server.repositories import RmfRepository
from api_server.rmf_io import get_rmf_events

router = FastIORouter(tags=["Lifts"])


@router.get("", response_model=list[Lift])
async def get_lifts(rmf_repo: Annotated[RmfRepository, Depends(RmfRepository)]):
    return await rmf_repo.get_lifts()


@router.get("/{lift_name}/state", response_model=LiftState)
async def get_lift_state(
    lift_name: str, rmf_repo: Annotated[RmfRepository, Depends(RmfRepository)]
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
    user = req.user
    obs = get_rmf_events().lift_states.pipe(
        rxops.filter(lambda x: x.lift_name == lift_name)
    )
    lift_state = await get_lift_state(lift_name, RmfRepository(user))
    if lift_state:
        return obs.pipe(rxops.start_with(lift_state))
    return obs


@router.post("/{lift_name}/request")
def post_lift_request(
    lift_name: str,
    lift_request: LiftRequest,
    rmf_gateway: Annotated[RmfGateway, Depends(get_rmf_gateway)],
):
    rmf_gateway.request_lift(
        lift_name,
        lift_request.destination,
        lift_request.request_type,
        lift_request.door_mode,
        lift_request.additional_session_ids,
    )
