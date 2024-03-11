# NOTE: This will eventually replace `gateway.py``
import os
from typing import Any, Dict

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from websockets.exceptions import ConnectionClosed

from api_server import models as mdl
from api_server.app_config import app_config
from api_server.logger import logger as base_logger
from api_server.repositories import AlertRepository, FleetRepository, TaskRepository
from api_server.rmf_io import alert_events, fleet_events, task_events

router = APIRouter(tags=["_internal"])
logger = base_logger.getChild("RmfGatewayApp")
user: mdl.User = mdl.User(username="__rmf_internal__", is_admin=True)
task_repo = TaskRepository(user)
alert_repo = AlertRepository(user, task_repo)


class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(
            f"ConnectionManager: {len(self.active_connections)} websocket connections still alive"
        )

        # Temporary fix for https://github.com/open-rmf/rmf-web/issues/897
        if (
            app_config.max_internal_websocket_connections is not None
            and len(self.active_connections)
            > app_config.max_internal_websocket_connections
        ):
            logger.error(
                f"ConnectionManager: exceeded maximum allowed internal websocket connections [{app_config.max_internal_websocket_connections}]"
            )
            logger.error("ConnectionManager: Shutting down server")
            os._exit(1)  # pylint: disable=protected-access

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        logger.info(
            f"ConnectionManager: {len(self.active_connections)} websocket connections still alive"
        )


connection_manager = ConnectionManager()


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


async def process_msg(msg: Dict[str, Any], fleet_repo: FleetRepository) -> None:
    if "type" not in msg:
        logger.warn(msg)
        logger.warn("Ignoring message, 'type' must include in msg field")
        return
    payload_type: str = msg["type"]
    if not isinstance(payload_type, str):
        logger.warn("error processing message, 'type' must be a string")
        return
    logger.debug(msg)

    if payload_type == "task_state_update":
        task_state = mdl.TaskState(**msg["data"])
        await task_repo.save_task_state(task_state)
        task_events.task_states.on_next(task_state)

        if (
            task_state.status == mdl.Status.completed
            or task_state.status == mdl.Status.failed
        ):
            alert = await alert_repo.create_alert(task_state.booking.id, "task")
            if alert is not None:
                alert_events.alerts.on_next(alert)
        # elif (
        #     task_state.unix_millis_finish_time
        #     and task_state.unix_millis_warn_time
        #     and task_state.unix_millis_finish_time > task_state.unix_millis_warn_time
        # ):
        #     # TODO(AC): Perhaps set a late alert as its own category
        #     late_alert_id = f"{task_state.booking.id}__late"
        #     if not await alert_repo.alert_original_id_exists(late_alert_id):
        #         alert = await alert_repo.create_alert(late_alert_id, "task")
        #         if alert is not None:
        #             alert_events.alerts.on_next(alert)

    elif payload_type == "task_log_update":
        task_log = mdl.TaskEventLog(**msg["data"])
        await task_repo.save_task_log(task_log)
        task_events.task_event_logs.on_next(task_log)

        # if task_log_has_error(task_log):
        #     alert = await alert_repo.create_alert(task_log.task_id, "task")
        #     if alert is not None:
        #         alert_events.alerts.on_next(alert)

    elif payload_type == "fleet_state_update":
        fleet_state = mdl.FleetState(**msg["data"])
        await fleet_repo.save_fleet_state(fleet_state)
        fleet_events.fleet_states.on_next(fleet_state)

    elif payload_type == "fleet_log_update":
        fleet_log = mdl.FleetLog(**msg["data"])
        await fleet_repo.save_fleet_log(fleet_log)
        fleet_events.fleet_logs.on_next(fleet_log)


@router.websocket("")
async def rmf_gateway(websocket: WebSocket):
    await connection_manager.connect(websocket)
    fleet_repo = FleetRepository(user)
    try:
        while True:
            msg: Dict[str, Any] = await websocket.receive_json()
            await process_msg(msg, fleet_repo)
    except (WebSocketDisconnect, ConnectionClosed):
        connection_manager.disconnect(websocket)
        logger.warn("Client websocket disconnected")
