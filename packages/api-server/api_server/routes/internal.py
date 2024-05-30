# NOTE: This will eventually replace `gateway.py``
import logging
import os
from datetime import datetime
from typing import Any, Dict
from uuid import uuid4

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect
from websockets.exceptions import ConnectionClosed

from api_server import models as mdl
from api_server.app_config import app_config
from api_server.logging import LoggerAdapter, get_logger
from api_server.repositories import AlertRepository, FleetRepository, TaskRepository
from api_server.rmf_io import alert_events, fleet_events, task_events

router = APIRouter(tags=["_internal"])
user: mdl.User = mdl.User(username="__rmf_internal__", is_admin=True)


class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logging.info(
            f"ConnectionManager: {len(self.active_connections)} websocket connections still alive"
        )

        # Temporary fix for https://github.com/open-rmf/rmf-web/issues/897
        if (
            app_config.max_internal_websocket_connections is not None
            and len(self.active_connections)
            > app_config.max_internal_websocket_connections
        ):
            logging.error(
                f"ConnectionManager: exceeded maximum allowed internal websocket connections [{app_config.max_internal_websocket_connections}]"
            )
            logging.error("ConnectionManager: Shutting down server")
            os._exit(1)  # pylint: disable=protected-access

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        logging.info(
            f"ConnectionManager: {len(self.active_connections)} websocket connections still alive"
        )


connection_manager = ConnectionManager()


async def process_msg(
    msg: Dict[str, Any],
    alert_repo: AlertRepository,
    fleet_repo: FleetRepository,
    task_repo: TaskRepository,
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

        if task_state.status == mdl.Status.completed:
            alert_request = mdl.AlertRequest(
                id=str(uuid4()),
                unix_millis_alert_time=round(datetime.now().timestamp() * 1000),
                title="Task completed",
                subtitle=f"ID: {task_state.booking.id}",
                message="",
                display=True,
                tier=mdl.AlertRequest.Tier.Info,
                responses_available=["Acknowledge"],
                alert_parameters=[],
                task_id=task_state.booking.id,
            )
            created_alert = await alert_repo.create_new_alert(alert_request)
            alert_events.alert_requests.on_next(created_alert)
        elif task_state.status == mdl.Status.failed:
            errorMessage = ""
            if (
                task_state.dispatch is not None
                and task_state.dispatch.status == mdl.Status1.failed_to_assign
            ):
                errorMessage += "Failed to assign\n"
                if task_state.dispatch.errors is not None:
                    for error in task_state.dispatch.errors:
                        errorMessage += error.json() + "\n"

            alert_request = mdl.AlertRequest(
                id=str(uuid4()),
                unix_millis_alert_time=round(datetime.now().timestamp() * 1000),
                title="Task failed",
                subtitle=f"ID: {task_state.booking.id}",
                message=errorMessage,
                display=True,
                tier=mdl.AlertRequest.Tier.Error,
                responses_available=["Acknowledge"],
                alert_parameters=[],
                task_id=task_state.booking.id,
            )
            created_alert = await alert_repo.create_new_alert(alert_request)
            alert_events.alert_requests.on_next(created_alert)

    elif payload_type == "task_log_update":
        task_log = mdl.TaskEventLog(**msg["data"])
        await task_repo.save_task_log(task_log)
        task_events.task_event_logs.on_next(task_log)

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
    logger: LoggerAdapter = Depends(get_logger),
):
    await connection_manager.connect(websocket)
    alert_repo = AlertRepository()
    fleet_repo = FleetRepository(user, logger)
    task_repo = TaskRepository(user, logger)
    try:
        while True:
            msg: Dict[str, Any] = await websocket.receive_json()
            await process_msg(msg, alert_repo, fleet_repo, task_repo, logger)
    except (WebSocketDisconnect, ConnectionClosed):
        connection_manager.disconnect(websocket)
        logger.warning("Client websocket disconnected")
