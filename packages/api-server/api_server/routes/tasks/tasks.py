from fastapi import APIRouter
from fastapi.responses import JSONResponse

from ...models import CancelTask, SubmitTask
from .dispatcher import DispatcherClient

router = APIRouter(tags=["tasks"])

dispatcher_client = DispatcherClient()


@router.get("/get_tasks")
async def get_tasks():
    tasks = await dispatcher_client.get_task_status()
    return JSONResponse(content=tasks)


@router.post("/submit_task")
async def submit_task(submit_task_params: SubmitTask):
    req_msg, err_msg = dispatcher_client.convert_task_request(submit_task_params)

    if req_msg:
        task_id = await dispatcher_client.submit_task_request(req_msg)
        if task_id:
            return JSONResponse(content={"task_id": task_id, "error_msg": ""})

    return JSONResponse(content={"error_msg": err_msg})


@router.post("/cancel_task")
async def cancel_task(task: CancelTask):
    cancel_status = await dispatcher_client.cancel_task_request(task)
    return JSONResponse(content={"success": cancel_status})
