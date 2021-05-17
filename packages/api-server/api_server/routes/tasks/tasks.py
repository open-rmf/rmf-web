from datetime import datetime
from typing import Callable, List, Optional

from fastapi import Depends, HTTPException
from fastapi.responses import JSONResponse

from api_server.models.tasks import TaskStateEnum, TaskTypeEnum

from ...fast_io import FastIORouter
from ...gateway import RmfGateway
from ...models import CancelTask, SubmitTask, SubmitTaskResponse, TaskProgress
from ...models import tortoise_models as ttm
from ...services.tasks import convert_task_request
from .dispatcher import DispatcherClient


class TasksRouter(FastIORouter):
    def __init__(
        self,
        rmf_gateway_dep: Callable[[], RmfGateway],
    ):
        super().__init__(tags=["Tasks"])
        _dispatcher_client: Optional[DispatcherClient] = None

        def dispatcher_client_dep():
            nonlocal _dispatcher_client
            if _dispatcher_client is None:
                _dispatcher_client = DispatcherClient(rmf_gateway_dep())
            return _dispatcher_client

        @self.get("", response_model=List[TaskProgress])
        async def get_tasks(
            dispatcher_client: DispatcherClient = Depends(dispatcher_client_dep),
            task_id: Optional[str] = None,
            fleet_name: Optional[str] = None,
            submission_time_since: Optional[datetime] = None,
            start_time_since: Optional[datetime] = None,
            end_time_since: Optional[datetime] = None,
            robot_name: Optional[str] = None,
            state: Optional[str] = None,
            task_type: Optional[str] = None,
            offset: int = 0,
        ):
            filter_params = {}
            if task_id is not None:
                filter_params["task_id"] = task_id
            if fleet_name is not None:
                filter_params["fleet_name"] = fleet_name
            if submission_time_since is not None:
                filter_params["submission_time__gte"] = submission_time_since
            if start_time_since is not None:
                filter_params["start_time__gte"] = start_time_since
            if end_time_since is not None:
                filter_params["end_time__gte"] = end_time_since
            if robot_name is not None:
                filter_params["robot_name"] = robot_name
            if state is not None:
                try:
                    filter_params["state"] = TaskStateEnum[state.upper()].value
                except KeyError as e:
                    raise HTTPException(422, "unknown state") from e
            if task_type is not None:
                try:
                    filter_params["task_type"] = TaskTypeEnum[task_type.upper()].value
                except KeyError as e:
                    raise HTTPException(422, "unknown task type") from e

            task_summaries = [
                result.to_pydantic()
                for result in await ttm.TaskSummary.filter(**filter_params)
                .limit(100)
                .offset(offset)
            ]
            return [
                dispatcher_client.convert_task_status_msg(summary)
                for summary in task_summaries
            ]

        @self.post("/submit_task", response_model=SubmitTaskResponse)
        async def submit_task(
            submit_task_params: SubmitTask,
            dispatcher_client: DispatcherClient = Depends(dispatcher_client_dep),
        ):
            req_msg, err_msg = convert_task_request(
                submit_task_params, dispatcher_client.get_sim_time()
            )

            if err_msg:
                raise HTTPException(422, err_msg)

            rmf_resp = await dispatcher_client.submit_task_request(req_msg)
            if not rmf_resp.success:
                raise HTTPException(422, rmf_resp.message)
            return {"task_id": rmf_resp.task_id}

        @self.post("/cancel_task")
        async def cancel_task(
            task: CancelTask,
            dispatcher_client: DispatcherClient = Depends(dispatcher_client_dep),
        ):
            cancel_status = await dispatcher_client.cancel_task_request(task)
            return JSONResponse(content={"success": cancel_status})
