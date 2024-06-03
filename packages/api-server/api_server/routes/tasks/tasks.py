from datetime import datetime
from typing import List, Optional, Tuple, cast

import tortoise.functions as tfuncs
from fastapi import Body, Depends, HTTPException, Path, Query
from reactivex import operators as rxops
from tortoise.expressions import Q

from api_server import models as mdl
from api_server.dependencies import (
    between_query,
    finish_time_between_query,
    pagination_query,
    sio_user,
    start_time_between_query,
)
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models.tortoise_models import TaskState as DbTaskState
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


@router.get("", response_model=List[mdl.TaskState])
async def query_task_states(
    task_repo: TaskRepository = Depends(task_repo_dep),
    task_id: Optional[str] = Query(
        None, description="comma separated list of task ids"
    ),
    category: Optional[str] = Query(
        None, description="comma separated list of task categories"
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
    label: str
    | None = Query(
        None,
        description="comma separated list of labels, each item must be in the form <key>=<value>, multiple items will filter tasks with all the labels",
    ),
    pagination: mdl.Pagination = Depends(pagination_query),
):
    filters = {}
    if task_id is not None:
        filters["id___in"] = task_id.split(",")
    if category is not None:
        filters["category__in"] = category.split(",")
    if assigned_to is not None:
        filters["assigned_to__in"] = assigned_to.split(",")
    if start_time_between is not None:
        filters["unix_millis_start_time__gte"] = start_time_between[0]
        filters["unix_millis_start_time__lte"] = start_time_between[1]
    if finish_time_between is not None:
        filters["unix_millis_finish_time__gte"] = finish_time_between[0]
        filters["unix_millis_finish_time__lte"] = finish_time_between[1]
    if status is not None:
        valid_values = [member.value for member in mdl.TaskStatus]
        filters["status__in"] = []
        for status_string in status.split(","):
            if status_string not in valid_values:
                continue
            filters["status__in"].append(mdl.TaskStatus(status_string))
    query = DbTaskState.filter(**filters)

    label_filters = {}
    if label is not None:
        labels = mdl.Labels.from_strings(label.split(","))
        label_filters.update(
            {
                f"label_filter_{k}": tfuncs.Count(
                    "id_", _filter=Q(labels__label_name=k, labels__label_value=v)
                )
                for k, v in labels.root.items()
            }
        )

    if len(label_filters) > 0:
        filter_gt = {f"{f}__gt": 0 for f in label_filters}
        query = (
            query.annotate(**label_filters)
            .group_by(
                "labels__state_id"
            )  # need to group by a related field to make tortoise-orm generate joins
            .filter(**filter_gt)
        )

    return await task_repo.query_task_states(query, pagination)


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
    obs = task_events.task_states.pipe(rxops.filter(lambda x: x.booking.id == task_id))
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
        rxops.filter(lambda x: x.task_id == task_id)
    )


@router.post("/activity_discovery", response_model=mdl.ActivityDiscovery)
async def post_activity_discovery(
    request: mdl.ActivityDiscoveryRequest = Body(...),
):
    return RawJSONResponse(
        await tasks_service().call(request.model_dump_json(exclude_none=True))
    )


@router.post("/cancel_task", response_model=mdl.TaskCancelResponse)
async def post_cancel_task(
    request: mdl.CancelTaskRequest = Body(...),
):
    return RawJSONResponse(
        await tasks_service().call(request.model_dump_json(exclude_none=True))
    )


@router.post(
    "/dispatch_task",
    response_model=mdl.TaskDispatchResponse,
    responses={400: {"model": mdl.TaskDispatchResponse}},
)
async def post_dispatch_task(
    request: mdl.DispatchTaskRequest = Body(...),
    task_repo: TaskRepository = Depends(task_repo_dep),
):
    resp = mdl.TaskDispatchResponse.model_validate_json(
        await tasks_service().call(request.model_dump_json(exclude_none=True))
    )
    if not resp.root.success:
        return RawJSONResponse(resp.model_dump_json(), 400)
    task_state = cast(mdl.TaskDispatchResponse1, resp.root).state
    await task_repo.save_task_state(task_state)
    await task_repo.save_task_request(task_state.booking.id, request.request)
    return resp


@router.post(
    "/robot_task",
    response_model=mdl.RobotTaskResponse,
    responses={400: {"model": mdl.RobotTaskResponse}},
)
async def post_robot_task(
    request: mdl.RobotTaskRequest = Body(...),
    task_repo: TaskRepository = Depends(task_repo_dep),
):
    resp = mdl.RobotTaskResponse.model_validate_json(
        await tasks_service().call(request.model_dump_json(exclude_none=True))
    )
    if not resp.root.root.success:
        return RawJSONResponse(resp.model_dump_json(), 400)
    await task_repo.save_task_state(
        cast(mdl.TaskDispatchResponse1, resp.root.root).state
    )
    return resp


@router.post("/interrupt_task", response_model=mdl.TaskInterruptionResponse)
async def post_interrupt_task(
    request: mdl.TaskInterruptionRequest = Body(...),
):
    return RawJSONResponse(
        await tasks_service().call(request.model_dump_json(exclude_none=True))
    )


@router.post("/kill_task", response_model=mdl.TaskKillResponse)
async def post_kill_task(
    request: mdl.TaskKillRequest = Body(...),
):
    return RawJSONResponse(
        await tasks_service().call(request.model_dump_json(exclude_none=True))
    )


@router.post("/resume_task", response_model=mdl.TaskResumeResponse)
async def post_resume_task(
    request: mdl.TaskResumeRequest = Body(...),
):
    return RawJSONResponse(
        await tasks_service().call(request.model_dump_json(exclude_none=True))
    )


@router.post("/rewind_task", response_model=mdl.TaskRewindResponse)
async def post_rewind_task(
    request: mdl.TaskRewindRequest = Body(...),
):
    return RawJSONResponse(
        await tasks_service().call(request.model_dump_json(exclude_none=True))
    )


@router.post("/skip_phase", response_model=mdl.SkipPhaseResponse)
async def post_skip_phase(
    request: mdl.TaskPhaseSkipRequest = Body(...),
):
    return RawJSONResponse(
        await tasks_service().call(request.model_dump_json(exclude_none=True))
    )


@router.post("/task_discovery", response_model=mdl.TaskDiscovery)
async def post_task_discovery(
    request: mdl.TaskDiscoveryRequest = Body(...),
):
    return RawJSONResponse(
        await tasks_service().call(request.model_dump_json(exclude_none=True))
    )


@router.post("/undo_skip_phase", response_model=mdl.UndoPhaseSkipResponse)
async def post_undo_skip_phase(
    request: mdl.UndoPhaseSkipRequest = Body(...),
):
    return RawJSONResponse(
        await tasks_service().call(request.model_dump_json(exclude_none=True))
    )
