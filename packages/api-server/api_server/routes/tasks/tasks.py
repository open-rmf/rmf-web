from datetime import datetime
from typing import List, Optional

from fastapi import Depends, HTTPException, Path
from fastapi.param_functions import Query
from fastapi.responses import JSONResponse
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.dependencies import AddPaginationQuery, pagination_query
from api_server.fast_io import FastIORouter, WatchRequest
from api_server.models import (
    CancelTask,
    SubmitTask,
    SubmitTaskResponse,
    Task,
    TaskStateEnum,
    TaskSummary,
    TaskTypeEnum,
    User,
)
from api_server.models import tortoise_models as ttm
from api_server.routes.tasks.dispatcher import DispatcherClient
from api_server.routes.tasks.utils import get_task_progress
from api_server.routes.utils import rx_watcher
from api_server.services.tasks import convert_task_request


class TasksRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        user_dep = app.auth_dep
        rmf_repo = app.rmf_repo

        super().__init__(tags=["Tasks"], user_dep=user_dep)
        _dispatcher_client: Optional[DispatcherClient] = None

        def dispatcher_client_dep():
            nonlocal _dispatcher_client
            if _dispatcher_client is None:
                _dispatcher_client = DispatcherClient(app.rmf_gateway())
            return _dispatcher_client

        @self.get("/{task_id}/summary", response_model=TaskSummary)
        async def get_task_summary(
            task_id: str = Path(..., description="task_id with '/' replaced with '__'"),
            user: User = Depends(user_dep),
        ):
            """
            Available in socket.io
            """
            # FIXME: This would fail if task_id contains "_/"
            task_id = task_id.replace("__", "/")
            ts = await rmf_repo.query_tasks(user).get_or_none(id_=task_id)
            if ts is None:
                raise HTTPException(404)
            return TaskSummary(**ts.data)

        @self.watch("/{task_id}/summary")
        async def watch_task_summary(req: WatchRequest, task_id: str):
            await req.emit(await get_task_summary(task_id))
            rx_watcher(
                req,
                app.rmf_events().task_summaries.pipe(
                    rxops.filter(lambda x: x.task_id == task_id)
                ),
            )

        def to_task(tt_summary: ttm.TaskSummary):
            py_summary = tt_summary.to_pydantic()
            return Task.construct(
                task_id=py_summary.task_id,
                authz_grp=tt_summary.authz_grp,
                progress=get_task_progress(
                    py_summary,
                    app.rmf_gateway().now(),
                ),
                summary=py_summary,
            )

        @self.get("", response_model=List[Task])
        async def get_tasks(
            user: User = Depends(user_dep),
            add_pagination: AddPaginationQuery[ttm.TaskSummary] = Depends(
                pagination_query({"task_id": "id_"})
            ),
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
            filter_params = {}
            if task_id is not None:
                filter_params["id___in"] = task_id.split(",")
            if fleet_name is not None:
                filter_params["fleet_name__in"] = fleet_name.split(",")
            if submission_time_since is not None:
                filter_params["submission_time__gte"] = submission_time_since
            if start_time_since is not None:
                filter_params["start_time__gte"] = start_time_since
            if end_time_since is not None:
                filter_params["end_time__gte"] = end_time_since
            if robot_name is not None:
                filter_params["robot_name__in"] = robot_name.split(",")
            if state is not None:
                try:
                    filter_params["state__in"] = [
                        TaskStateEnum[s.upper()].value for s in state.split(",")
                    ]
                except KeyError as e:
                    raise HTTPException(422, "unknown state") from e
            if task_type is not None:
                try:
                    filter_params["task_type__in"] = [
                        TaskTypeEnum[t.upper()].value for t in task_type.split(",")
                    ]
                except KeyError as e:
                    raise HTTPException(422, "unknown task type") from e
            if priority is not None:
                filter_params["priority"] = priority

            q = rmf_repo.query_tasks(user).filter(**filter_params)
            task_summaries = await add_pagination(q)
            return [to_task(ts) for ts in task_summaries]

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
