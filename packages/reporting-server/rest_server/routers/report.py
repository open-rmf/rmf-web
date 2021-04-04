from fastapi import APIRouter
from repository.raw_log import get_raw_logs
from repository.report import get_doors_state

router = APIRouter()


@router.get("/door_state/", tags=["door_state"])
async def door_state_report():
    return get_doors_state()


@router.get("/raw_logs/", tags=["raw_logs"])
async def raw_logs_report():
    return get_raw_logs()
