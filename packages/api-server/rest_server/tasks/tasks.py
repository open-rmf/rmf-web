from typing import Optional

import rclpy
from fastapi import APIRouter
from fastapi.encoders import jsonable_encoder
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from .dispatcher import DispatcherClient

router = APIRouter(prefix="/tasks", tags=["tasks"])

rclpy.init(args=None)
dispatcher_client = DispatcherClient()


class Description(BaseModel):
    cleaning_zone: Optional[str] = None
    option: Optional[str] = None
    pickup_place_name: Optional[str] = None
    pickup_dispenser: Optional[str] = None
    dropoff_ingestor: Optional[str] = None
    dropoff_place_name: Optional[str] = None
    start_name: Optional[str] = None
    finish_name: Optional[str] = None
    num_loops: Optional[int] = None


class SubmitTask(BaseModel):
    task_type: str
    start_time: int
    priority: Optional[int] = None
    description: Description


class TaskId(BaseModel):
    task_id: str


@router.get("/get_tasks")
async def get_tasks():
    tasks = dispatcher_client.get_task_status()
    return JSONResponse(content=tasks)


@router.post("/submit_task")
async def submit_task(submit_task_params: SubmitTask):
    params_to_dict = jsonable_encoder(submit_task_params)
    print(params_to_dict)
    req_msg, err_msg = dispatcher_client.convert_task_request(params_to_dict)

    if req_msg:
        task_id = dispatcher_client.submit_task_request(req_msg)
        if task_id:
            return JSONResponse(content={"task_id": task_id, "error_msg": ""})

    return JSONResponse(content={"error_msg": err_msg})


@router.post("/cancel_task")
async def cancel_task(task_id: TaskId):
    request_to_task_id = jsonable_encoder(task_id)
    get_task_id = request_to_task_id["task_id"]
    cancel_status = dispatcher_client.cancel_task_request(get_task_id)
    return JSONResponse(content={"success": cancel_status})
