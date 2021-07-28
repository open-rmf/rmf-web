from typing import Any, List, Optional

from fastapi import APIRouter, HTTPException, status
from rest_server.repositories.log_creation_handler import RawLogHandler

router = APIRouter()

# This will receive information from different sources
LIMIT = 500


@router.get("/", tags=["all_scheduled_tasks"], response_model=List[Any])
async def get_scheduled_task(
    offset: Optional[int] = 0,
    limit: Optional[int] = LIMIT,
):
    try:
        return await get_scheduled_tasks(offset, limit)
    except Exception as e:
        print(e)
        raise HTTPException(503, "cannot create the log" + str(e)) from e


@router.post("/", tags=["create_scheduled_task"], status_code=status.HTTP_201_CREATED)
async def create_scheduled_task(body: dict):
    try:
        response = await create_rmf_server_log(body)
        if not isinstance(response, str):
            raise HTTPException(503, "Error creating some logs" + str(response))

        return response

    except Exception as e:
        print(e)
        raise HTTPException(503, "cannot create the rmfserver log" + str(e)) from e


# Will receive information from keycloak only
@router.delete("/", tags=["delete_scheduled_task"], status_code=status.HTTP_201_CREATED)
async def delete_scheduled_task(id: int):
    try:
        response = await create_keycloak_log(body)
        if not isinstance(response, str):
            raise HTTPException(503, "Error creating some logs" + str(response))

        return response

    except Exception as e:
        print(e)
        raise HTTPException(503, "cannot create the keycloak log" + str(e)) from e
