from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel
from rest_server.repositories.log_creation_handler import (
    create_raw_log,
    create_rmf_server_log,
)

router = APIRouter()


# This will receive information from different sources
@router.post("/all/", tags=["all_logs"], status_code=status.HTTP_201_CREATED)
async def write_logs(body: list):
    print("request body", body)
    try:
        await create_raw_log(body)
    except Exception as e:
        print(e)
        raise HTTPException(503, "cannot create the log" + str(e))


# Will receive information from rmf-server only
@router.post(
    "/rmfserver/", tags=["rmfserver_logs"], status_code=status.HTTP_201_CREATED
)
async def write_rmf_server_logs(body: list):
    print("request body", body)
    try:
        await create_rmf_server_log(body)
    except Exception as e:
        print(e)
        raise HTTPException(503, "cannot create the rmfserver log" + str(e))
