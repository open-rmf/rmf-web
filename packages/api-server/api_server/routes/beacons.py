from typing import List

from fastapi import Depends, HTTPException
from rx import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import tortoise_models as ttm
from api_server.rmf_io import beacon_events

router = FastIORouter(tags=["Beacons"])


@router.sub("", response_model=ttm.BeaconStatePydantic)
async def sub_beacons(_req: SubscriptionRequest):
    return beacon_events.alerts.pipe(rxops.filter(lambda x: x is not None))


@router.get("", response_model=List[ttm.BeaconStatePydantic])
async def get_beacons():
    beacons = await ttm.BeaconState.all()
    return [await ttm.BeaconStatePydantic.from_tortoise_orm(a) for a in beacons]


@router.get("/{id}", response_model=ttm.BeaconStatePydantic)
async def get_beacon(id: str):
    beacon_state = await ttm.BeaconState.get_or_none(id=id)
    if beacon_state is None:
        logger.error(f"No existing beacon with ID {id}")
        return None
    beacon_state_pydantic = await ttm.BeaconStatePydantic.from_tortoise_orm(
        beacon_state
    )
    return beacon_state_pydantic


@router.post("", status_code=201, response_model=ttm.BeaconStatePydantic)
async def save_beacon_state(
    id: str, online: bool, category: str, activated: bool, level: str
):
    beacon_state, _ = await ttm.BeaconState.update_or_create(
        {
            "online": online,
            "category": category,
            "activated": activated,
            "level": level,
        },
        id=id,
    )
    if beacon_state is None:
        logger.error(f"Failed to save beacon state with ID {id}")
        return None
    beacon_state_pydantic = await ttm.BeaconStatePydantic.from_tortoise_orm(
        beacon_state
    )
    beacon_events.beacons.on_next(beacon_state_pydantic)
    return beacon_state_pydantic
