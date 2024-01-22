from typing import List

from fastapi import HTTPException
from reactivex import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import tortoise_models as ttm
from api_server.rmf_io import beacon_events

router = FastIORouter(tags=["Beacons"])


@router.sub("", response_model=ttm.BeaconStatePydantic)
async def sub_beacons(_req: SubscriptionRequest):
    return beacon_events.beacons.pipe(rxops.filter(lambda x: x is not None))


@router.get("", response_model=List[ttm.BeaconStatePydantic])
async def get_beacons():
    beacons = await ttm.BeaconState.all()
    return [await ttm.BeaconStatePydantic.from_tortoise_orm(a) for a in beacons]


@router.get("/{beacon_id}", response_model=ttm.BeaconStatePydantic)
async def get_beacon(beacon_id: str):
    beacon_state = await ttm.BeaconState.get_or_none(id=beacon_id)
    if beacon_state is None:
        raise HTTPException(404, f"Beacon with ID {beacon_id} not found")
    beacon_state_pydantic = await ttm.BeaconStatePydantic.from_tortoise_orm(
        beacon_state
    )
    return beacon_state_pydantic


@router.post("", status_code=201, response_model=ttm.BeaconStatePydantic)
async def save_beacon_state(
    beacon_id: str, online: bool, category: str, activated: bool, level: str
):
    beacon_state, _ = await ttm.BeaconState.update_or_create(
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
    beacon_state_pydantic = await ttm.BeaconStatePydantic.from_tortoise_orm(
        beacon_state
    )
    beacon_events.beacons.on_next(beacon_state_pydantic)
    return beacon_state_pydantic
