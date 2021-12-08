from datetime import datetime
from typing import List, Optional, cast

from fastapi import Depends, HTTPException, Path
from fastapi.param_functions import Query
from fastapi.responses import JSONResponse
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.dependencies import pagination_query
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import (
    CancelTask,
    SubmitTask,
    SubmitTaskResponse,
    Task,
    TaskSummary,
    User,
)
from api_server.models.pagination import Pagination
from api_server.repositories import RmfRepository
from api_server.services.tasks import convert_task_request

from .dispatcher import DispatcherClient
from .utils import get_task_progress


class TasksRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        user_dep = app.user_dep

        super().__init__(tags=["Tasks"])
        _dispatcher_client: Optional[DispatcherClient] = None

        def dispatcher_client_dep():
            nonlocal _dispatcher_client
            if _dispatcher_client is None:
                _dispatcher_client = DispatcherClient(app.rmf_gateway())
            return _dispatcher_client

        @self.get("/{task_id}/summary", response_model=TaskSummary)
        async def get_task_summary(
            rmf_repo: RmfRepository = Depends(app.rmf_repo),
            task_id: str = Path(..., description="task_id with '/' replaced with '__'"),
        ):
            """
            Available in socket.io
            """
            ts = await rmf_repo.get_task_summary(task_id)
            return ts.dict(exclude_none=True)

        @self.sub("/{task_id}/summary", response_model=TaskSummary)
        async def sub_task_summary(req: SubscriptionRequest, task_id: str):
            user = req.session["user"]
            try:
                await req.sio.emit(
                    req.room,
                    await get_task_summary(RmfRepository(user), task_id),
                    req.sid,
                )
            except HTTPException:
                pass
            return app.rmf_events().task_summaries.pipe(
                rxops.filter(lambda x: cast(TaskSummary, x).task_id == task_id)
            )

        def to_task(task_summary: TaskSummary):
            return Task.construct(
                task_id=task_summary.task_id,
                authz_grp=task_summary.authz_grp,
                progress=get_task_progress(
                    task_summary,
                    app.rmf_gateway().now(),
                ),
                summary=task_summary,
            )

        @self.get("", response_model=List[Task])
        async def get_tasks(
            rmf_repo: RmfRepository = Depends(app.rmf_repo),
            pagination: Pagination = Depends(pagination_query),
            task_id: Optional[str] = Query(
                None, description="comma separated list of task ids"
            ),
            fleet_name: Optional[str] = Query(
                None, description="comma separated list of fleet names"
            ),
            submission_time_since: Optional[datetime] = None,
            start_time_since: Optional[datetime] = None,
            end_time_since: Optional[datetime] = None,
            robot_name: Optional[str] = Query(
                None, description="comma separated list of robot names"
            ),
            state: Optional[str] = Query(
                None, description="comma separated list of states"
            ),
            task_type: Optional[str] = Query(
                None, description="comma separated list of task types"
            ),
            priority: Optional[int] = None,
        ):
            task_summaries = await rmf_repo.query_task_summaries(
                pagination,
                task_id=task_id,
                fleet_name=fleet_name,
                submission_time_since=submission_time_since,
                start_time_since=start_time_since,
                end_time_since=end_time_since,
                robot_name=robot_name,
                state=state,
                task_type=task_type,
                priority=priority,
            )
            return [to_task(t) for t in task_summaries]

        @self.post("/submit_task", response_model=SubmitTaskResponse)
        async def submit_task(
            submit_task_params: SubmitTask,
            user: User = Depends(user_dep),
            dispatcher_client: DispatcherClient = Depends(dispatcher_client_dep),
        ):
            req_msg, err_msg = convert_task_request(
                submit_task_params, app.rmf_gateway().now()
            )

            if err_msg:
                raise HTTPException(422, err_msg)

            rmf_resp = await dispatcher_client.submit_task_request(user, req_msg)
            if not rmf_resp.success:
                raise HTTPException(422, rmf_resp.message)

            return {"task_id": rmf_resp.task_id}

        @self.post("/cancel_task")
        async def cancel_task(
            task: CancelTask,
            user: User = Depends(user_dep),
            dispatcher_client: DispatcherClient = Depends(dispatcher_client_dep),
        ):
            cancel_status = await dispatcher_client.cancel_task_request(task, user)
            return JSONResponse(content={"success": cancel_status})
