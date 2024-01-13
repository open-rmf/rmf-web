# NOTE: This will eventually replace `gateway.py``
import os
from datetime import datetime
from typing import Any, Dict

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from api_server import models as mdl
from api_server.logger import logger as base_logger
from api_server.repositories import AlertRepository, FleetRepository, TaskRepository
from api_server.rmf_io import alert_events, fleet_events, task_events

router = APIRouter(tags=["_internal"])
logger = base_logger.getChild("RmfGatewayApp")
user: mdl.User = mdl.User(username="__rmf_internal__", is_admin=True)
task_repo = TaskRepository(user)
alert_repo = AlertRepository(user, task_repo)


class WebSocketHealthManager:
    def __init__(self):
        self.disconnects = []

    def disconnected(self):
        self.disconnects.append(datetime.now())

        # Clean up if the past disconnection is more than 2 minutes ago
        if len(self.disconnects) > 2:
            seconds_since_last_disconnect = (
                self.disconnects[-1] - self.disconnects[-2]
            ).seconds
            logger.warn(
                f"Previous Web Socket disconnection was {seconds_since_last_disconnect} seconds ago"
            )
            if seconds_since_last_disconnect > 120:
                logger.info(
                    "Previous Web Socket disconnection was more than 2 minutes ago, cleaning up"
                )
                self.disconnects = [datetime.now()]

        # If there are more than 5 disconnects that occurred within 2 minutes, shut down the server
        if len(self.disconnects) > 5:
            unhealthy_period_seconds = (
                self.disconnects[-1] - self.disconnects[-5]
            ).seconds
            if unhealthy_period_seconds < 120:
                logger.error(
                    f"Web Sockets had 5 disconnections within {unhealthy_period_seconds} seconds"
                )
                logger.error("Shutting down server")
                os._exit(1)  # pylint: disable=protected-access


health_manager = WebSocketHealthManager()


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

        if task_state.status == mdl.Status.completed:
            alert = await alert_repo.create_alert(task_state.booking.id, "task")
            if alert is not None:
                alert_events.alerts.on_next(alert)
        elif (
            task_state.unix_millis_finish_time
            and task_state.unix_millis_warn_time
            and task_state.unix_millis_finish_time > task_state.unix_millis_warn_time
        ):
            # TODO(AC): Perhaps set a late alert as its own category
            late_alert_id = f"{task_state.booking.id}__late"
            if not await alert_repo.alert_original_id_exists(late_alert_id):
                alert = await alert_repo.create_alert(late_alert_id, "task")
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
async def rmf_gateway(websocket: WebSocket):
    await websocket.accept()
    fleet_repo = FleetRepository(user)
    try:
        while True:
            msg: Dict[str, Any] = await websocket.receive_json()
            await process_msg(msg, fleet_repo)
    except WebSocketDisconnect:
        health_manager.disconnected()
        logger.warn("Client websocket disconnected")
