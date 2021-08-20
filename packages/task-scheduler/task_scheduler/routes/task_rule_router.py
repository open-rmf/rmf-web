from typing import Any, List, Optional

from fastapi import APIRouter, HTTPException, status

from task_scheduler.models.pydantic_models import TaskRule_Pydantic
from task_scheduler.repositories import TaskRuleRepository

router = APIRouter()

LIMIT = 500


@router.get("/", tags=["all_rules"], response_model=List[TaskRule_Pydantic])
async def get_task_rules(
    offset: Optional[int] = 0,
    limit: Optional[int] = LIMIT,
):
    try:
        return await TaskRuleRepository.get(offset, limit)
    except Exception as e:
        print(e)
        raise HTTPException(503, "Cannot retrieving task rules:" + str(e)) from e


@router.post(
    "/",
    tags=["create_rule"],
    status_code=status.HTTP_201_CREATED,
    response_model=TaskRule_Pydantic,
)
async def create_scheduled_task(body: dict):
    try:
        return await TaskRuleRepository.create(body)
    except Exception as e:
        raise HTTPException(503, "Cannot create the task rule" + str(e)) from e


@router.delete("/", tags=["delete_rule"], status_code=status.HTTP_204_NO_CONTENT)
async def delete_scheduled_task(id: int):
    try:
        await TaskRuleRepository.delete(id)

    except Exception as e:
        print(e)
        raise HTTPException(503, "Cannot delete task rule " + str(e)) from e
