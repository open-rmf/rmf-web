from typing import Annotated, List, cast

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.dependencies import sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import RmfGateway
from api_server.models import Door, DoorRequest, DoorState
from api_server.repositories import RmfRepository
from api_server.rmf_io import RmfEvents

router = FastIORouter(tags=["Doors"])


@router.get("", response_model=List[Door])
async def get_doors(rmf_repo: RmfRepository = Depends(RmfRepository)):
    return await rmf_repo.get_doors()


@router.get("/{door_name}/state", response_model=DoorState)
async def get_door_state(
    door_name: str, rmf_repo: RmfRepository = Depends(RmfRepository)
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
    obs = RmfEvents.get_instance().door_states.pipe(
        rxops.filter(lambda x: cast(DoorState, x).door_name == door_name)
    )
    door_state = await get_door_state(door_name, RmfRepository(user))
    if door_state:
        return obs.pipe(rxops.start_with(door_state))
    return obs


@router.post("/{door_name}/request")
def post_door_request(
    door_name: str,
    door_request: DoorRequest,
    rmf_gateway: Annotated[RmfGateway, Depends(RmfGateway.get_instance)],
):
    rmf_gateway.request_door(door_name, door_request.mode)
