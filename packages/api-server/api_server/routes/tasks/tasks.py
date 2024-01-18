from datetime import datetime
from typing import List, Optional, Tuple, cast

from fastapi import Body, Depends, HTTPException, Path, Query
from rx import operators as rxops

from api_server import models as mdl
from api_server.dependencies import (
    between_query,
    finish_time_between_query,
    pagination_query,
    request_time_between_query,
    sio_user,
    start_time_between_query,
)
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models.tortoise_models import TaskQueueEntryPydantic
from api_server.repositories import TaskRepository, task_repo_dep
from api_server.response import RawJSONResponse
from api_server.rmf_io import task_events, tasks_service

router = FastIORouter(tags=["Tasks"])


@router.get("/{task_id}/request", response_model=mdl.TaskRequest)
async def get_task_request(
    task_repo: TaskRepository = Depends(task_repo_dep),
    task_id: str = Path(..., description="task_id"),
):
    result = await task_repo.get_task_request(task_id)
    if result is None:
        raise HTTPException(status_code=404)
    return result


@router.get("/requests", response_model=List[Optional[mdl.TaskRequest]])
async def query_task_requests(
    task_repo: TaskRepository = Depends(task_repo_dep),
    task_ids: Optional[str] = Query(
        None, description="comma separated list of task ids"
    ),
):
    task_id_splits = []
    if task_ids is not None:
        task_id_splits = task_ids.split(",")
    valid_task_requests = await task_repo.query_task_requests(task_id_splits)

    valid_task_request_map = {}
    for valid_req in valid_task_requests:
        valid_task_request_map[valid_req.id_] = mdl.TaskRequest(**valid_req.request)

    return_requests = []
    for id_query in task_id_splits:
        if id_query in valid_task_request_map:
            return_requests.append(valid_task_request_map[id_query])
        else:
            return_requests.append(None)
    return return_requests


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
    obs = task_events.task_states.pipe(
        rxops.filter(lambda x: cast(mdl.TaskState, x).booking.id == task_id)
    )
    current_state = await get_task_state(task_repo, task_id)
    if current_state:
        return obs.pipe(rxops.start_with(current_state))
    return obs


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
    return RawJSONResponse(await tasks_service().call(request.json(exclude_none=True)))


@router.post("/cancel_task", response_model=mdl.TaskCancelResponse)
async def post_cancel_task(
    request: mdl.CancelTaskRequest = Body(...),
):
    return RawJSONResponse(await tasks_service().call(request.json(exclude_none=True)))


@router.post(
    "/dispatch_task",
    response_model=mdl.TaskDispatchResponseItem,
    responses={400: {"model": mdl.TaskDispatchResponseItem1}},
)
async def post_dispatch_task(
    request: mdl.DispatchTaskRequest = Body(...),
    task_repo: TaskRepository = Depends(task_repo_dep),
):
    task_warn_time = request.request.unix_millis_warn_time
    resp = mdl.TaskDispatchResponse.parse_raw(
        await tasks_service().call(request.json(exclude_none=True))
    )
    if not resp.__root__.success:
        return RawJSONResponse(resp.json(), 400)
    new_state = cast(mdl.TaskDispatchResponseItem, resp.__root__).state
    if task_warn_time is not None:
        new_state.unix_millis_warn_time = task_warn_time
    await task_repo.save_task_state(new_state)
    await task_repo.save_task_request(new_state, request.request)
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
        await tasks_service().call(request.json(exclude_none=True))
    )
    if not resp.__root__.__root__.success:
        return RawJSONResponse(resp.json(), 400)
    await task_repo.save_task_state(
        cast(mdl.TaskDispatchResponseItem, resp.__root__.__root__).state
    )
    return resp.__root__


@router.post("/interrupt_task", response_model=mdl.TaskInterruptionResponse)
async def post_interrupt_task(
    request: mdl.TaskInterruptionRequest = Body(...),
):
    return RawJSONResponse(await tasks_service().call(request.json(exclude_none=True)))


@router.post("/kill_task", response_model=mdl.TaskKillResponse)
async def post_kill_task(
    request: mdl.TaskKillRequest = Body(...),
):
    return RawJSONResponse(await tasks_service().call(request.json(exclude_none=True)))


@router.post("/resume_task", response_model=mdl.TaskResumeResponse)
async def post_resume_task(
    request: mdl.TaskResumeRequest = Body(...),
):
    return RawJSONResponse(await tasks_service().call(request.json(exclude_none=True)))


@router.post("/rewind_task", response_model=mdl.TaskRewindResponse)
async def post_rewind_task(
    request: mdl.TaskRewindRequest = Body(...),
):
    return RawJSONResponse(await tasks_service().call(request.json(exclude_none=True)))


@router.post("/skip_phase", response_model=mdl.SkipPhaseResponse)
async def post_skip_phase(
    request: mdl.TaskPhaseSkipRequest = Body(...),
):
    return RawJSONResponse(await tasks_service().call(request.json(exclude_none=True)))


@router.post("/task_discovery", response_model=mdl.TaskDiscovery)
async def post_task_discovery(
    request: mdl.TaskDiscoveryRequest = Body(...),
):
    return RawJSONResponse(await tasks_service().call(request.json(exclude_none=True)))


@router.post("/undo_skip_phase", response_model=mdl.UndoPhaseSkipResponse)
async def post_undo_skip_phase(
    request: mdl.UndoPhaseSkipRequest = Body(...),
):
    return RawJSONResponse(await tasks_service().call(request.json(exclude_none=True)))


@router.get("/queue_entry", response_model=List[TaskQueueEntryPydantic])
async def query_task_queue_entry(
    task_repo: TaskRepository = Depends(task_repo_dep),
    task_id: Optional[str] = Query(
        None, description="comma separated list of task ids"
    ),
    category: Optional[str] = Query(
        None, description="comma separated list of task categories"
    ),
    request_time_between: Optional[Tuple[datetime, datetime]] = Depends(
        request_time_between_query
    ),
    requester: Optional[str] = Query(
        None, description="comma separated list of requester names"
    ),
    pickup: Optional[str] = Query(
        None, description="comma separated list of pickup names"
    ),
    destination: Optional[str] = Query(
        None, description="comma separated list of destination names"
    ),
    assigned_to: Optional[str] = Query(
        None, description="comma separated list of assigned robot names"
    ),
    start_time_between: Optional[Tuple[datetime, datetime]] = Depends(
        start_time_between_query
    ),
    finish_time_between: Optional[Tuple[datetime, datetime]] = Depends(
        finish_time_between_query
    ),
    status: Optional[str] = Query(None, description="comma separated list of statuses"),
    pagination: mdl.Pagination = Depends(pagination_query),
):
    filters = {}
    if task_id is not None:
        filters["id___in"] = task_id.split(",")
    if category is not None:
        filters["category__in"] = category.split(",")
    if request_time_between is not None:
        filters["unix_millis_request_time__gte"] = request_time_between[0]
        filters["unix_millis_request_time__lte"] = request_time_between[1]
    if requester is not None:
        filters["requester__in"] = requester.split(",")
    if pickup is not None:
        filters["pickup__in"] = pickup.split(",")
    if destination is not None:
        filters["destination__in"] = destination.split(",")
    if assigned_to is not None:
        filters["assigned_to__in"] = assigned_to.split(",")
    if start_time_between is not None:
        filters["unix_millis_start_time__gte"] = start_time_between[0]
        filters["unix_millis_start_time__lte"] = start_time_between[1]
    if finish_time_between is not None:
        filters["unix_millis_finish_time__gte"] = finish_time_between[0]
        filters["unix_millis_finish_time__lte"] = finish_time_between[1]
    if status is not None:
        valid_values = [member.value for member in mdl.Status]
        filters["status__in"] = []
        for status_string in status.split(","):
            if status_string not in valid_values:
                continue
            filters["status__in"].append(mdl.Status(status_string))

    return await task_repo.query_task_queue_entry(filters, pagination)


@router.get("/{task_id}/queue_entry", response_model=TaskQueueEntryPydantic)
async def get_task_queue_entry(
    task_repo: TaskRepository = Depends(task_repo_dep),
    task_id: str = Path(..., description="task_id"),
):
    """
    Available in socket.io
    """
    result = await task_repo.get_task_queue_entry(task_id)
    if result is None:
        raise HTTPException(status_code=404)
    return result


@router.sub("/{task_id}/queue_entry", response_model=TaskQueueEntryPydantic)
async def sub_task_queue_entry(req: SubscriptionRequest, task_id: str):
    user = sio_user(req)
    task_repo = TaskRepository(user)
    obs = task_events.task_queue_entries.pipe(
        rxops.filter(lambda x: cast(TaskQueueEntryPydantic, x).id_ == task_id)
    )
    current_task_queue_entry = await get_task_queue_entry(task_repo, task_id)
    if current_task_queue_entry:
        return obs.pipe(rxops.start_with(current_task_queue_entry))
    return obs
