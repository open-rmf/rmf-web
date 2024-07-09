from datetime import datetime
from typing import Annotated

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import RmfGateway, get_rmf_gateway
from api_server.logging import LoggerAdapter, get_logger
from api_server.models import BuildingMap, FireAlarmTriggerState
from api_server.repositories import RmfRepository
from api_server.rmf_io import get_rmf_events

router = FastIORouter(tags=["Building"])


@router.get("", response_model=BuildingMap)
async def get_building_map(rmf_repo: Annotated[RmfRepository, Depends(RmfRepository)]):
    """
    Available in socket.io
    """
    building_map = await rmf_repo.get_bulding_map()
    if building_map is None:
        raise HTTPException(status_code=404)
    return building_map


@router.sub("", response_model=BuildingMap)
def sub_building_map(_req: SubscriptionRequest):
    return get_rmf_events().building_map.pipe(rxops.filter(lambda x: x is not None))


@router.sub("/fire_alarm_trigger", response_model=FireAlarmTriggerState)
async def sub_fire_alarm_trigger(_req: SubscriptionRequest):
    return get_rmf_events().fire_alarm_trigger.pipe(
        rxops.filter(lambda x: x is not None)
    )


@router.get("/previous_fire_alarm_trigger", response_model=FireAlarmTriggerState)
async def get_previous_fire_alarm_trigger():
    previous_trigger = get_rmf_events().fire_alarm_trigger.value
    if previous_trigger is None:
        raise HTTPException(404, "previous fire alarm trigger not available")
    return previous_trigger


@router.post("/reset_fire_alarm_trigger", response_model=FireAlarmTriggerState)
async def reset_fire_alarm_trigger(
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
    rmf_gateway: Annotated[RmfGateway, Depends(get_rmf_gateway)],
):
    # TODO: enforce with authz
    logger.info("Fire alarm trigger reset requested")
    rmf_gateway.reset_fire_alarm_trigger()
    fire_alarm_trigger_state = FireAlarmTriggerState(
        unix_millis_time=round(datetime.now().timestamp() * 1000), trigger=False
    )
    return fire_alarm_trigger_state
