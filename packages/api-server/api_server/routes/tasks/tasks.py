from datetime import datetime
from typing import List, Optional, Tuple, cast

from fastapi import Body, Depends, HTTPException, Path, Query
from rx import operators as rxops
from rx.subject.replaysubject import ReplaySubject

from api_server import models as mdl
from api_server.dependencies import between_query, pagination_query, sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models.tortoise_models import TaskState as DbTaskState
from api_server.repositories import TaskRepository, task_repo_dep
from api_server.response import RawJSONResponse
from api_server.rmf_io import task_events, tasks_service

router = FastIORouter(tags=["Tasks"])


@router.get("", response_model=List[mdl.TaskState])
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
    pagination: mdl.Pagination = Depends(pagination_query),
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


@router.get("/{task_id}/state", response_model=mdl.TaskState)
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


@router.sub("/{task_id}/state", response_model=mdl.TaskState)
async def sub_task_state(req: SubscriptionRequest, task_id: str):
    user = sio_user(req)
    task_repo = TaskRepository(user)
    current_state = await get_task_state(task_repo, task_id)
    sub = ReplaySubject(1)
    if current_state:
        sub.on_next(current_state)
    task_events.task_states.pipe(
        rxops.filter(lambda x: cast(mdl.TaskState, x).booking.id == task_id)
    ).subscribe(sub)
    return sub


@router.get("/{task_id}/log", response_model=mdl.TaskEventLog)
async def get_task_log(
    task_repo: TaskRepository = Depends(task_repo_dep),
    task_id: str = Path(..., description="task_id"),
    between: Tuple[int, int] = Depends(between_query),
):
    """
    Available in socket.io
    """

    result = await task_repo.get_task_log(task_id, between)
    if result is None:
        raise HTTPException(status_code=404)
    return result


@router.sub("/{task_id}/log", response_model=mdl.TaskEventLog)
async def sub_task_log(_req: SubscriptionRequest, task_id: str):
    return task_events.task_event_logs.pipe(
        rxops.filter(lambda x: cast(mdl.TaskEventLog, x).task_id == task_id)
    )


@router.post("/activity_discovery", response_model=mdl.ActivityDiscovery)
async def post_activity_discovery(
    request: mdl.ActivityDiscoveryRequest = Body(...),
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/cancel_task", response_model=mdl.TaskCancelResponse)
async def post_cancel_task(
    request: mdl.CancelTaskRequest = Body(...),
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post(
    "/dispatch_task",
    response_model=mdl.TaskDispatchResponseItem,
    responses={400: {"model": mdl.TaskDispatchResponseItem1}},
)
async def post_dispatch_task(
    request: mdl.DispatchTaskRequest = Body(...),
    task_repo: TaskRepository = Depends(task_repo_dep),
):
    resp = mdl.TaskDispatchResponse.parse_raw(
        await tasks_service.call(request.json(exclude_none=True))
    )
    if not resp.__root__.success:
        return RawJSONResponse(resp.json(), 400)
    await task_repo.save_task_state(
        cast(mdl.TaskDispatchResponseItem, resp.__root__).state
    )
    return resp.__root__


@router.post(
    "/robot_task",
    response_model=mdl.RobotTaskResponse,
    responses={400: {"model": mdl.RobotTaskResponse}},
)
async def post_robot_task(
    request: mdl.RobotTaskRequest = Body(...),
    task_repo: TaskRepository = Depends(task_repo_dep),
):
    resp = mdl.RobotTaskResponse.parse_raw(
        await tasks_service.call(request.json(exclude_none=True))
    )
    if not resp.__root__.__root__.success:
        return RawJSONResponse(resp.json(), 400)
    await task_repo.save_task_state(
        cast(mdl.TaskDispatchResponseItem, resp.__root__).state
    )
    return resp.__root__


@router.post("/interrupt_task", response_model=mdl.TaskInterruptionResponse)
async def post_interrupt_task(
    request: mdl.TaskInterruptionRequest = Body(...),
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/kill_task", response_model=mdl.TaskKillResponse)
async def post_kill_task(
    request: mdl.TaskKillRequest = Body(...),
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/resume_task", response_model=mdl.TaskResumeResponse)
async def post_resume_task(
    request: mdl.TaskResumeRequest = Body(...),
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/rewind_task", response_model=mdl.TaskRewindResponse)
async def post_rewind_task(
    request: mdl.TaskRewindRequest = Body(...),
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/skip_phase", response_model=mdl.SkipPhaseResponse)
async def post_skip_phase(
    request: mdl.TaskPhaseSkipRequest = Body(...),
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/task_discovery", response_model=mdl.TaskDiscovery)
async def post_task_discovery(
    request: mdl.TaskDiscoveryRequest = Body(...),
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/undo_skip_phase", response_model=mdl.UndoPhaseSkipResponse)
async def post_undo_skip_phase(
    request: mdl.UndoPhaseSkipRequest = Body(...),
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))
