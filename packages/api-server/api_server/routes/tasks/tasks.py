from datetime import datetime
from typing import Any, List, Optional, cast

from fastapi import Body, Depends, HTTPException, Path, Query
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.dependencies import pagination_query, task_repo_dep
from api_server.fast_io import FastIORouter, WatchRequest
from api_server.models import (
    CancelTaskRequest,
    Pagination,
    TaskCancelResponse,
    TaskRequest,
    TaskState,
    User,
)
from api_server.models.tortoise_models import TaskState as DbTaskState
from api_server.query import add_pagination
from api_server.repositories import TaskRepository

from ..utils import rx_watcher


class TasksRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        user_dep = app.auth_dep

        super().__init__(tags=["Tasks"], user_dep=user_dep)

        @self.get("", response_model=List[TaskState])
        async def query_task_states(
            task_repo: TaskRepository = Depends(task_repo_dep(app)),
            task_id: Optional[str] = Query(
                None, description="comma separated list of task ids"
            ),
            category: Optional[str] = Query(
                None, description="comma separated list of task categories"
            ),
            start_time: Optional[datetime] = None,
            finish_time: Optional[datetime] = None,
            pagination: Pagination = Depends(pagination_query),
        ):
            filters = {}
            if task_id is not None:
                filters["id___in"] = task_id.split(",")
            if category is not None:
                filters["category__in"] = category.split(",")
            if start_time is not None:
                filters["unix_millis_start_time__gte"] = start_time
            if finish_time is not None:
                filters["unix_millis_finish_time__gte"] = finish_time

            return await task_repo.get_task_states(
                add_pagination(DbTaskState.filter(**filters), pagination)
            )

        @self.get("/{task_id}/state", response_model=TaskState)
        async def get_task_state(
            task_repo: TaskRepository = Depends(task_repo_dep(app)),
            task_id: str = Path(..., description="task_id"),
        ):
            """
            Available in socket.io
            """
            results = await task_repo.get_task_states(DbTaskState.filter(id_=task_id))
            if not results:
                raise HTTPException(status_code=404)
            return results[0]

        @self.watch("/{task_id}/state")
        async def watch_task_state(req: WatchRequest, task_id: str):
            task_repo = TaskRepository(req.user)
            try:
                current_state = await get_task_state(task_repo, task_id)
                await req.emit(current_state.json())
            except HTTPException as e:
                if e.status_code != 404:
                    raise e

            rx_watcher(
                req,
                app.rmf_events().task_states.pipe(
                    rxops.filter(lambda x: cast(TaskState, x).booking.id == task_id),
                    rxops.map(cast(Any, lambda x: cast(TaskState, x).json())),
                ),
            )

        @self.post("/task_request")
        async def task_request(
            user: User = Depends(user_dep),
            task_request: TaskRequest = Body(...),
        ):
            # TODO: forward to the internal app
            raise HTTPException(status_code=501)

        @self.post("/cancel_task", response_model=TaskCancelResponse)
        async def cancel_task(
            user: User = Depends(user_dep),
            cancel_request: CancelTaskRequest = Body(...),
        ):
            # TODO: forward to the internal app
            raise HTTPException(status_code=501)
