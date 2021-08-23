from typing import Any, List, Optional

from fastapi import APIRouter, HTTPException, status

from task_scheduler.repositories import ScheduledTaskRepository

router = APIRouter()

LIMIT = 500


@router.get("/", tags=["all_scheduled_tasks"], response_model=List[Any])
async def get_scheduled_task(
    offset: Optional[int] = 0,
    limit: Optional[int] = LIMIT,
):
    try:
        return await ScheduledTaskRepository.get(offset, limit)
    except Exception as e:
        raise HTTPException(503, "Cannot get scheduled tasks" + str(e)) from e


# Will receive information from keycloak only
@router.delete(
    "/{scheduled_task_id}",
    tags=["delete_scheduled_task"],
    status_code=status.HTTP_204_NO_CONTENT,
)
async def delete_scheduled_task(scheduled_task_id: int):
    try:
        await ScheduledTaskRepository.delete(scheduled_task_id)

    except Exception as e:
        raise HTTPException(503, "Cannot delete the scheduled task" + str(e)) from e
