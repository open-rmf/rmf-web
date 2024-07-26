from typing import Annotated

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import Dispenser, DispenserState
from api_server.repositories import RmfRepository
from api_server.rmf_io import get_rmf_events

router = FastIORouter(tags=["Dispensers"])


@router.get("", response_model=list[Dispenser])
async def get_dispensers(rmf_repo: Annotated[RmfRepository, Depends(RmfRepository)]):
    return await rmf_repo.get_dispensers()


@router.get("/{guid}/state", response_model=DispenserState)
async def get_dispenser_state(
    guid: str, rmf_repo: Annotated[RmfRepository, Depends(RmfRepository)]
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
    user = req.user
    obs = get_rmf_events().dispenser_states.pipe(rxops.filter(lambda x: x.guid == guid))
    dispenser_state = await get_dispenser_state(guid, RmfRepository(user))
    if dispenser_state:
        return obs.pipe(rxops.start_with(dispenser_state))
    return obs
