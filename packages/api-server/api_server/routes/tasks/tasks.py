from datetime import datetime
from typing import Annotated, List, Optional, Tuple, cast

from fastapi import Body, Depends, HTTPException, Path, Query
from reactivex import operators as rxops

from api_server import models as mdl
from api_server.dependencies import (
    between_query,
    pagination_query,
    sio_user,
    time_between_query,
)
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.logging import LoggerAdapter, default_logger, get_logger
from api_server.repositories import RmfRepository, TaskRepository
from api_server.response import RawJSONResponse
from api_server.rmf_io import TaskEvents, TasksService
from api_server.routes.building_map import get_building_map

router = FastIORouter(tags=["Tasks"])


async def cancellation_lots_from_building_map(
    logger: LoggerAdapter, rmf_repo: RmfRepository
) -> List[str]:
    building_map = None
    try:
        building_map = await get_building_map(rmf_repo)
    except HTTPException:
        logger.warning(
            "No building map found, cannot obtain emergency lots for cancellation behavior"
        )
        return []

    cancellation_lots = []
    for level in building_map.levels:
        for graph in level.nav_graphs:
            for node in graph.vertices:
                # FIXME: Use properties to determine if a node is a cancellation
                # behavior lot, instead of substring matching.
                if "emergency" in node.name.lower():
                    cancellation_lots.append(node.name)
    logger.info(f"Cancellation lots found: {cancellation_lots}")
    return cancellation_lots


@router.get("/{task_id}/request", response_model=mdl.TaskRequest)
async def get_task_request(
    task_repo: TaskRepository = Depends(TaskRepository),
    task_id: str = Path(..., description="task_id"),
):
    result = await task_repo.get_task_request(task_id)
    if result is None:
        raise HTTPException(status_code=404)
    return result


@router.get("/requests", response_model=List[Optional[mdl.TaskRequest]])
async def query_task_requests(
    task_repo: TaskRepository = Depends(TaskRepository),
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


@router.get("", response_model=List[mdl.TaskState])
async def query_task_states(
    task_repo: TaskRepository = Depends(TaskRepository),
    task_id: Optional[str] = Query(
        None, description="comma separated list of task ids"
    ),
    category: Optional[str] = Query(
        None, description="comma separated list of task categories"
    ),
    request_time_between: Optional[Tuple[datetime, datetime]] = Depends(
        time_between_query
    ),
    requester: Optional[str] = Query(
        None, description="comma separated list of requester names"
    ),
    assigned_to: Optional[str] = Query(
        None, description="comma separated list of assigned robot names"
    ),
    start_time_between: Optional[Tuple[datetime, datetime]] = Depends(
        time_between_query
    ),
    finish_time_between: Optional[Tuple[datetime, datetime]] = Depends(
        time_between_query
    ),
    status: Optional[str] = Query(None, description="comma separated list of statuses"),
    label: str | None = Query(
        None,
        description="comma separated list of labels, each item must be in the form <key>=<value>, multiple items will filter tasks with all the labels",
    ),
    pagination: mdl.Pagination = Depends(pagination_query),
):
    labels = mdl.Labels.from_strings(label.split(",")) if label else mdl.Labels(root={})

    return await task_repo.query_task_states(
        task_id=task_id.split(",") if task_id else None,
        category=category.split(",") if category else None,
        assigned_to=assigned_to.split(",") if assigned_to else None,
        start_time_between=start_time_between,
        finish_time_between=finish_time_between,
        request_time_between=request_time_between,
        requester=requester.split(",") if requester else None,
        status=status.split(",") if status else None,
        label=labels,
        pagination=pagination,
    )


@router.get("/{task_id}/state", response_model=mdl.TaskState)
async def get_task_state(
    task_repo: TaskRepository = Depends(TaskRepository),
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
    task_repo = TaskRepository(user, default_logger)
    obs = TaskEvents.get_instance().task_states.pipe(
        rxops.filter(lambda x: cast(mdl.TaskState, x).booking.id == task_id)
    )
    current_state = await get_task_state(task_repo, task_id)
    if current_state:
        return obs.pipe(rxops.start_with(current_state))
    return obs


@router.get("/{task_id}/booking_label", response_model=mdl.Labels)
async def get_task_booking_label(
    task_repo: TaskRepository = Depends(TaskRepository),
    task_id: str = Path(..., description="task_id"),
):
    state = await task_repo.get_task_state(task_id)
    if state is None:
        raise HTTPException(status_code=404)

    if state.booking.labels is not None:
        for label in state.booking.labels:
            if len(label) == 0:
                continue

            booking_label = mdl.Labels.from_strings(label)
            if booking_label is not None:
                return booking_label
    raise HTTPException(status_code=404)


@router.get("/{task_id}/log", response_model=mdl.TaskEventLog)
async def get_task_log(
    task_repo: TaskRepository = Depends(TaskRepository),
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
    return TaskEvents.get_instance().task_event_logs.pipe(
        rxops.filter(lambda x: cast(mdl.TaskEventLog, x).task_id == task_id)
    )


@router.post("/activity_discovery", response_model=mdl.ActivityDiscovery)
async def post_activity_discovery(
    request: Annotated[mdl.ActivityDiscoveryRequest, Body(...)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/cancel_task", response_model=mdl.TaskCancelResponse)
async def post_cancel_task(
    request: Annotated[mdl.CancelTaskRequest, Body(...)],
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
):
    logger.info(request)
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post(
    "/dispatch_task",
    response_model=mdl.TaskDispatchResponse,
    responses={400: {"model": mdl.TaskDispatchResponse}},
)
async def post_dispatch_task(
    request: Annotated[mdl.DispatchTaskRequest, Body(...)],
    rmf_repo: Annotated[RmfRepository, Depends(RmfRepository)],
    task_repo: Annotated[TaskRepository, Depends(TaskRepository)],
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
):
    # FIXME: In order to accommodate changing cancellation lots over time, and
    # avoiding updating all the saved scheduled tasks in the database, we only
    # insert cancellation lots as part of the cancellation behavior before
    # dispatching.
    # Check if request even has cancellation behavior.
    if (
        request.request.category == "compose"
        and request.request.description is not None
        and "phases" in request.request.description
        and len(request.request.description["phases"]) == 3
        and "on_cancel" in request.request.description["phases"][1]
    ):
        cancellation_lots = await cancellation_lots_from_building_map(logger, rmf_repo)
        if len(cancellation_lots) != 0:
            # Populate them in the correct form
            go_to_one_of_the_places_activity = {
                "category": "go_to_place",
                "description": {
                    "one_of": [{"waypoint": name} for name in cancellation_lots],
                    "constraints": [{"category": "prefer_same_map", "description": ""}],
                },
            }
            delivery_dropoff_activity = {
                "category": "perform_action",
                "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_dropoff",
                    "description": {},
                },
            }
            on_cancel_dropoff = {
                "category": "sequence",
                "description": [
                    go_to_one_of_the_places_activity,
                    delivery_dropoff_activity,
                ],
            }
            # Add into task request
            request.request.description["phases"][1]["on_cancel"] = [on_cancel_dropoff]

    resp = mdl.TaskDispatchResponse.model_validate_json(
        await tasks_service.call(request.model_dump_json(exclude_none=True))
    )
    logger.info(resp)
    if not resp.root.success:
        return RawJSONResponse(resp.model_dump_json(), 400)
    new_state = cast(mdl.TaskDispatchResponse1, resp.root).state
    await task_repo.save_task_state(new_state)
    await task_repo.save_task_request(new_state, request.request)
    return resp.root


@router.post(
    "/robot_task",
    response_model=mdl.RobotTaskResponse,
    responses={400: {"model": mdl.RobotTaskResponse}},
)
async def post_robot_task(
    request: Annotated[mdl.RobotTaskRequest, Body(...)],
    task_repo: Annotated[TaskRepository, Depends(TaskRepository)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
):
    resp = mdl.RobotTaskResponse.parse_raw(
        await tasks_service.call(request.json(exclude_none=True))
    )
    if not resp.root.root.success:
        return RawJSONResponse(resp.json(), 400)
    await task_repo.save_task_state(
        cast(mdl.TaskDispatchResponse1, resp.root.root).state
    )
    return resp.root


@router.post("/interrupt_task", response_model=mdl.TaskInterruptionResponse)
async def post_interrupt_task(
    request: Annotated[mdl.TaskInterruptionRequest, Body(...)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/kill_task", response_model=mdl.TaskKillResponse)
async def post_kill_task(
    request: Annotated[mdl.TaskKillRequest, Body(...)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/resume_task", response_model=mdl.TaskResumeResponse)
async def post_resume_task(
    request: Annotated[mdl.TaskResumeRequest, Body(...)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/rewind_task", response_model=mdl.TaskRewindResponse)
async def post_rewind_task(
    request: Annotated[mdl.TaskRewindRequest, Body(...)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/skip_phase", response_model=mdl.SkipPhaseResponse)
async def post_skip_phase(
    request: Annotated[mdl.TaskPhaseSkipRequest, Body(...)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/task_discovery", response_model=mdl.TaskDiscovery)
async def post_task_discovery(
    request: Annotated[mdl.TaskDiscoveryRequest, Body(...)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))


@router.post("/undo_skip_phase", response_model=mdl.UndoPhaseSkipResponse)
async def post_undo_skip_phase(
    request: Annotated[mdl.UndoPhaseSkipRequest, Body(...)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
):
    return RawJSONResponse(await tasks_service.call(request.json(exclude_none=True)))
