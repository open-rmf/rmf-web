import rclpy
from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse

from .dispatcher import DispatcherClient

router = APIRouter(prefix="/tasks", tags=["tasks"])

rclpy.init(args=None)
dispatcher_client = DispatcherClient()


@router.get("/task_list")
async def get_tasks():
    tasks = dispatcher_client.get_task_status()
    return JSONResponse(content=tasks)


@router.post("/submit_task")
async def submit_task(submit_task_params: Request):
    params_to_dict = await submit_task_params.json()
    req_msg, err_msg = dispatcher_client.convert_task_request(params_to_dict)

    if req_msg:
        task_id = dispatcher_client.submit_task_request(req_msg)
        if task_id:
            return JSONResponse(content={"task_id": task_id, "error_msg": ""})

    return JSONResponse(content={"error_msg": err_msg})
