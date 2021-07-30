from typing import Any, List, Optional

from fastapi import APIRouter, HTTPException, status

from task_scheduler.models.pydantic_models import TaskRule_Pydantic

router = APIRouter()

# This will receive information from different sources
LIMIT = 500


@router.get("/", tags=["all_rules"], response_model=List[TaskRule_Pydantic])
async def get_scheduled_task(
    offset: Optional[int] = 0,
    limit: Optional[int] = LIMIT,
):
    try:
        return await get_scheduled_tasks(offset, limit)
    except Exception as e:
        print(e)
        raise HTTPException(503, "cannot create the log" + str(e)) from e


@router.post(
    "/",
    tags=["create_rule"],
    status_code=status.HTTP_201_CREATED,
    response_model=TaskRule_Pydantic,
)
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
@router.delete("/", tags=["delete_rule"], status_code=status.HTTP_201_CREATED)
async def delete_scheduled_task(id: int):
    try:
        response = await create_keycloak_log(body)
        if not isinstance(response, str):
            raise HTTPException(503, "Error creating some logs" + str(response))

        return response

    except Exception as e:
        print(e)
        raise HTTPException(503, "cannot create the keycloak log" + str(e)) from e
