# NOTE: This will eventually replace `gateway.py``
from typing import Annotated, Any

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect
from websockets.exceptions import ConnectionClosed

from api_server import models as mdl
from api_server.logging import LoggerAdapter, get_logger
from api_server.models.user import User
from api_server.repositories import AlertRepository, FleetRepository, TaskRepository
from api_server.rmf_io import AlertEvents, FleetEvents, TaskEvents
from api_server.rmf_io.events import get_alert_events, get_fleet_events, get_task_events

router = APIRouter(tags=["_internal"])


def log_phase_has_error(phase: mdl.Phases) -> bool:
    if phase.log:
        for log in phase.log:
            if log.tier == mdl.Tier.error:
                return True
    if phase.events:
        for _, event_logs in phase.events.items():
            for event_log in event_logs:
                if event_log.tier == mdl.Tier.error:
                    return True
    return False


def task_log_has_error(task_log: mdl.TaskEventLog) -> bool:
    if task_log.log:
        for log in task_log.log:
            if log.tier == mdl.Tier.error:
                return True

    if task_log.phases:
        for _, phase in task_log.phases.items():
            if log_phase_has_error(phase):
                return True
    return False


async def process_msg(
    msg: dict[str, Any],
    fleet_repo: FleetRepository,
    task_repo: TaskRepository,
    alert_repo: AlertRepository,
    task_events: TaskEvents,
    alert_events: AlertEvents,
    fleet_events: FleetEvents,
    logger: LoggerAdapter,
) -> None:
    if "type" not in msg:
        logger.warning(msg)
        logger.warning("Ignoring message, 'type' must include in msg field")
        return
    payload_type: str = msg["type"]
    if not isinstance(payload_type, str):
        logger.warning("error processing message, 'type' must be a string")
        return
    logger.debug(msg)

    if payload_type == "task_state_update":
        task_state = mdl.TaskState(**msg["data"])
        await task_repo.save_task_state(task_state)
        task_events.task_states.on_next(task_state)

        if task_state.status in (mdl.TaskStatus.completed, mdl.TaskStatus.failed):
            alert = await alert_repo.create_alert(task_state.booking.id, "task")
            if alert is not None:
                alert_events.alerts.on_next(alert)

    elif payload_type == "task_log_update":
        task_log = mdl.TaskEventLog(**msg["data"])
        await task_repo.save_task_log(task_log)
        task_events.task_event_logs.on_next(task_log)

        if task_log_has_error(task_log):
            alert = await alert_repo.create_alert(task_log.task_id, "task")
            if alert is not None:
                alert_events.alerts.on_next(alert)

    elif payload_type == "fleet_state_update":
        fleet_state = mdl.FleetState(**msg["data"])
        await fleet_repo.save_fleet_state(fleet_state)
        fleet_events.fleet_states.on_next(fleet_state)

    elif payload_type == "fleet_log_update":
        fleet_log = mdl.FleetLog(**msg["data"])
        await fleet_repo.save_fleet_log(fleet_log)
        fleet_events.fleet_logs.on_next(fleet_log)


@router.websocket("")
async def rmf_gateway(
    websocket: WebSocket,
    task_events: Annotated[TaskEvents, Depends(get_task_events)],
    alert_events: Annotated[AlertEvents, Depends(get_alert_events)],
    fleet_events: Annotated[FleetEvents, Depends(get_fleet_events)],
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
):
    # We must resolve some dependencies manually because:
    # 1. `user_dep` uses `OpenIdConnect` which does not work for websocket
    # 2. Even if it works, the _internal route has no authentication
    user = User.get_system_user()
    fleet_repo = FleetRepository(user, logger)
    task_repo = TaskRepository(user, logger)
    alert_repo = AlertRepository(user, task_repo)

    await websocket.accept()
    try:
        while True:
            msg: dict[str, Any] = await websocket.receive_json()
            await process_msg(
                msg,
                fleet_repo,
                task_repo,
                alert_repo,
                task_events,
                alert_events,
                fleet_events,
                logger,
            )
    except (WebSocketDisconnect, ConnectionClosed):
        logger.warning("Client websocket disconnected")
