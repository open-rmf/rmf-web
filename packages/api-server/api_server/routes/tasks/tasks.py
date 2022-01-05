from datetime import datetime
from typing import List, Optional, cast

from fastapi import Body, Depends, HTTPException, Path, Query
from rx import operators as rxops

from api_server.authenticator import user_dep
from api_server.dependencies import pagination_query, sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import (
    CancelTaskRequest,
    Pagination,
    TaskCancelResponse,
    TaskRequest,
    TaskState,
    User,
)
from api_server.models.tasks import TaskEventLog
from api_server.models.tortoise_models import TaskState as DbTaskState
from api_server.repositories import TaskRepository, task_repo_dep
from api_server.rmf_io import task_events
from api_server.time import now

router = FastIORouter(tags=["Tasks"])


@router.get("", response_model=List[TaskState])
async def query_task_states(
    task_repo: TaskRepository = Depends(task_repo_dep),
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

    return await task_repo.query_task_states(DbTaskState.filter(**filters), pagination)


@router.get("/{task_id}/state", response_model=TaskState)
async def get_task_state(
    task_repo: TaskRepository = Depends(task_repo_dep),
    task_id: str = Path(..., description="task_id"),
):
    """
    Available in socket.io
    """
    result = await task_repo.get_task_state(task_id)
    if result is None:
        raise HTTPException(status_code=404)
    return result


@router.sub("/{task_id}/state", response_model=TaskState)
async def sub_task_state(req: SubscriptionRequest, task_id: str):
    user = sio_user(req)
    task_repo = TaskRepository(user)
    current_state = await get_task_state(task_repo, task_id)
    await req.sio.emit(req.room, current_state, req.sid)
    return task_events.task_states.pipe(
        rxops.filter(lambda x: cast(TaskState, x).booking.id == task_id)
    )


@router.get("/{task_id}/log", response_model=TaskEventLog)
async def get_task_log(
    task_repo: TaskRepository = Depends(task_repo_dep),
    task_id: str = Path(..., description="task_id"),
    between: str = Query(
        "-60000",
        description="""
        The period of time to fetch, in unix millis.

        This can be either a comma separated string or a string prefixed with '-' to fetch the last X millis.

        Example:
            "1000,2000" - Fetches logs between unix millis 1000 and 2000.
            "-60000" - Fetches logs in the last minute.
        """,
    ),
    now: int = Depends(now),
):
    """
    Available in socket.io
    """
    if between.startswith("-"):
        period = (now - int(between[1:]), now)
    else:
        parts = between.split(",")
        period = (int(parts[0]), int(parts[1]))
    result = await task_repo.get_task_log(task_id, period)
    if result is None:
        raise HTTPException(status_code=404)
    return result


@router.sub("/{task_id}/log", response_model=TaskEventLog)
async def sub_task_log(_req: SubscriptionRequest, task_id: str):
    return task_events.task_event_logs.pipe(
        rxops.filter(lambda x: cast(TaskEventLog, x).task_id == task_id)
    )


@router.post("/task_request")
async def post_task_request(
    user: User = Depends(user_dep),
    task_request: TaskRequest = Body(...),
):
    # TODO: forward to the internal app
    raise HTTPException(status_code=501)


@router.post("/cancel_task", response_model=TaskCancelResponse)
async def post_cancel_task(
    user: User = Depends(user_dep),
    cancel_request: CancelTaskRequest = Body(...),
):
    # TODO: forward to the internal app
    raise HTTPException(status_code=501)
