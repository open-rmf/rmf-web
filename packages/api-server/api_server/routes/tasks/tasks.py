from typing import Callable, List, Optional

from fastapi import Depends, HTTPException
from fastapi.responses import JSONResponse

from api_server.repositories.rmf import RmfRepository

from ...fast_io import FastIORouter
from ...gateway import RmfGateway
from ...models import CancelTask, SubmitTask, SubmitTaskResponse, TaskProgress
from ...services.tasks import convert_task_request
from .dispatcher import DispatcherClient


class TasksRouter(FastIORouter):
    def __init__(
        self,
        rmf_repo: RmfRepository,
        rmf_gateway_dep: Callable[[], RmfGateway],
    ):
        super().__init__(tags=["Tasks"])
        _dispatcher_client: Optional[DispatcherClient] = None

        def dispatcher_client_dep():
            nonlocal _dispatcher_client
            if _dispatcher_client is None:
                _dispatcher_client = DispatcherClient(rmf_repo, rmf_gateway_dep())
            return _dispatcher_client

        @self.get("", response_model=List[TaskProgress])
        async def get_tasks(
            dispatcher_client: DispatcherClient = Depends(dispatcher_client_dep),
            task_id: Optional[str] = None,
            state: Optional[str] = None,
            offset: int = 0,
        ):
            task_summaries = await rmf_repo.query_task_summaries(
                task_id=task_id, state=state, offset=offset
            )
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
