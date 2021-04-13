from typing import List, Optional

from fastapi import APIRouter
from models.raw_log import RawLog_Pydantic
from rest_server.repositories.report import get_all_raw_logs, get_doors_state

router = APIRouter()


@router.get("/door_state/", tags=["door_state"])
async def door_state_report():
    return get_doors_state()


@router.get("/raw_logs/", tags=["raw_logs"], response_model=List[RawLog_Pydantic])
async def raw_logs_report(
    toLogDate: Optional[str] = None,
    fromLogDate: Optional[str] = None,
    logLabel: Optional[str] = None,
    logLevel: Optional[str] = None,
):

    return await get_all_raw_logs(toLogDate, fromLogDate, logLabel, logLevel)
