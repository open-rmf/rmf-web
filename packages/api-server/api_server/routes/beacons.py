from typing import Annotated

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import BeaconState
from api_server.models.tortoise_models import BeaconState as DbBeaconState
from api_server.rmf_io import RmfEvents, get_rmf_events

router = FastIORouter(tags=["Beacons"])


@router.sub("", response_model=BeaconState)
async def sub_beacons(_req: SubscriptionRequest):
    return get_rmf_events().beacons.pipe(rxops.filter(lambda x: x is not None))


@router.get("", response_model=list[BeaconState])
async def get_beacons():
    beacons = await DbBeaconState.all()
    return [BeaconState.model_validate(a) for a in beacons]


@router.get("/{beacon_id}", response_model=BeaconState)
async def get_beacon(beacon_id: str):
    beacon_state = await DbBeaconState.get_or_none(id=beacon_id)
    if beacon_state is None:
        raise HTTPException(404, f"Beacon with ID {beacon_id} not found")
    beacon_state_pydantic = BeaconState.model_validate(beacon_state)
    return beacon_state_pydantic


@router.post("", status_code=201, response_model=BeaconState)
async def save_beacon_state(
    beacon_id: str,
    online: bool,
    category: str,
    activated: bool,
    level: str,
    rmf_events: Annotated[RmfEvents, Depends(get_rmf_events)],
):
    # FIXME(koonpeng): this is not publishing to rmf
    beacon_state, _ = await DbBeaconState.update_or_create(
        {
            "online": online,
            "category": category,
            "activated": activated,
            "level": level,
        },
        id=beacon_id,
    )
    if beacon_state is None:
        raise HTTPException(404, f"Could not save beacon state with ID {beacon_id}")
    beacon_state_pydantic = BeaconState.model_validate(beacon_state)
    rmf_events.beacons.on_next(beacon_state_pydantic)
    return beacon_state_pydantic
