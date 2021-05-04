from typing import List, Optional

from fastapi import APIRouter
from models.dispenser_state import DispenserState_Pydantic
from models.door_state import DoorState_Pydantic
from models.fleet_state import FleetState_Pydantic
from models.health import HealthStatus_Pydantic
from models.ingestor_state import IngestorState_Pydantic
from models.lift_state import LiftState_Pydantic
from models.raw_log import RawLog_Pydantic
from rest_server.repositories.report import (
    get_all_raw_logs,
    get_containers,
    get_dispenser_state,
    get_door_state,
    get_fleet_state,
    get_health,
    get_ingestor_state,
    get_lift_state,
)

router = APIRouter()

LIMIT = 500


@router.get("/raw_logs/", tags=["raw_logs"], response_model=List[RawLog_Pydantic])
async def raw_logs_report(
    toLogDate: Optional[str] = None,
    fromLogDate: Optional[str] = None,
    logLevel: Optional[str] = None,
    offset: Optional[int] = 0,
    limit: Optional[int] = LIMIT,
):

    return await get_all_raw_logs(offset, limit, toLogDate, fromLogDate, logLevel)


@router.get("/raw_logs/containers", tags=["raw_logs_get_containers"])
async def raw_logs_get_containers():
    return await get_containers()


@router.get(
    "/door_state/", tags=["door_state"], response_model=List[DoorState_Pydantic]
)
async def door_state_report(
    toLogDate: Optional[str] = None,
    fromLogDate: Optional[str] = None,
    offset: Optional[int] = 0,
    limit: Optional[int] = LIMIT,
):

    return await get_door_state(offset, limit, toLogDate, fromLogDate)


@router.get(
    "/fleet_state/", tags=["fleet_state"], response_model=List[FleetState_Pydantic]
)
async def fleet_state_report(
    toLogDate: Optional[str] = None,
    fromLogDate: Optional[str] = None,
    offset: Optional[int] = 0,
    limit: Optional[int] = LIMIT,
):

    return await get_fleet_state(offset, limit, toLogDate, fromLogDate)


@router.get(
    "/dispenser_state/",
    tags=["dispenser_state"],
    response_model=List[DispenserState_Pydantic],
)
async def dispenser_state_report(
    toLogDate: Optional[str] = None,
    fromLogDate: Optional[str] = None,
    offset: Optional[int] = 0,
    limit: Optional[int] = LIMIT,
):

    return await get_dispenser_state(offset, limit, toLogDate, fromLogDate)


@router.get(
    "/ingestor_state/",
    tags=["ingestor_state"],
    response_model=List[IngestorState_Pydantic],
)
async def ingestor_state_report(
    toLogDate: Optional[str] = None,
    fromLogDate: Optional[str] = None,
    offset: Optional[int] = 0,
    limit: Optional[int] = LIMIT,
):

    return await get_ingestor_state(offset, limit, toLogDate, fromLogDate)


@router.get("/lift_state/", tags=["raw_logs"], response_model=List[LiftState_Pydantic])
async def lift_state_report(
    toLogDate: Optional[str] = None,
    fromLogDate: Optional[str] = None,
    offset: Optional[int] = 0,
    limit: Optional[int] = LIMIT,
):

    return await get_lift_state(offset, limit, toLogDate, fromLogDate)


@router.get("/health/", tags=["health"], response_model=List[HealthStatus_Pydantic])
async def health_report(
    toLogDate: Optional[str] = None,
    fromLogDate: Optional[str] = None,
    offset: Optional[int] = 0,
    limit: Optional[int] = LIMIT,
):

    return await get_health(offset, limit, toLogDate, fromLogDate)
