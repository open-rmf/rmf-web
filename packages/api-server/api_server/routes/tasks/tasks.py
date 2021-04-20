from typing import List

from fastapi import APIRouter
from fastapi.exceptions import HTTPException
from fastapi.responses import JSONResponse

from ...models import CancelTask, SubmitTask, SubmitTaskResponse, Task
from .dispatcher import DispatcherClient

router = APIRouter(tags=["tasks"])

dispatcher_client = DispatcherClient()


@router.get("/get_tasks", response_model=List[Task])
async def get_tasks():
    tasks = await dispatcher_client.get_task_status()
    return tasks


@router.post("/submit_task", response_model=SubmitTaskResponse)
async def submit_task(submit_task_params: SubmitTask):
    req_msg, err_msg = dispatcher_client.convert_task_request(submit_task_params)

    if err_msg:
        raise HTTPException(422, err_msg)

    rmf_resp = await dispatcher_client.submit_task_request(req_msg)
    if not rmf_resp.success:
        raise HTTPException(422, rmf_resp.message)
    return {"task_id": rmf_resp.task_id}


@router.post("/cancel_task")
async def cancel_task(task: CancelTask):
    cancel_status = await dispatcher_client.cancel_task_request(task)
    return JSONResponse(content={"success": cancel_status})
