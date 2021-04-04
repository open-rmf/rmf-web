from fastapi import APIRouter, status
from repository.raw_log import create_raw_log

router = APIRouter()


@router.post("/raw/", tags=["raw_logs"], status_code=status.HTTP_201_CREATED)
async def write_logs():
    create_raw_log()
