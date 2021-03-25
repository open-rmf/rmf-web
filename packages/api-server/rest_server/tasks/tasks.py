from typing import Optional

from fastapi import APIRouter
from pydantic import BaseModel

router = APIRouter(prefix="/tasks", tags=["tasks"])


class Description(BaseModel):
    start_name: str
    finish_name: str
    num_loops: int


class Submit_Task(BaseModel):
    task_type: str
    start_time: int
    description: Description


@router.post("/submit_task")
async def submit_task(submit_task_params: Submit_Task):
    return submit_task_params
