from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel
from rest_server.repositories.log_creation_handler import create_raw_log

router = APIRouter()

# class Data(BaseModel):
#     hola: str


@router.post("/raw/", tags=["raw_logs"], status_code=status.HTTP_201_CREATED)
async def write_logs(body: list):
    print("request body", body)
    try:
        await create_raw_log(body)
    except Exception as e:
        print(e)
        raise HTTPException(503, "cannot create the log")
