# NOTE: This will eventually replace `gateway.py``
from datetime import datetime
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from api_server import models as mdl
from api_server.logger import logger as base_logger
from api_server.models import tortoise_models as ttm
from api_server.repositories import FleetRepository, TaskRepository
from api_server.rmf_io import alert_events, fleet_events, task_events

router = APIRouter(tags=["_internal"])
logger = base_logger.getChild("RmfGatewayApp")
user: mdl.User = mdl.User(username="__rmf_internal__", is_admin=True)
task_repo = TaskRepository(user)


def task_log_has_error(task_log: mdl.TaskEventLog) -> bool:
    if task_log.log:
        for log in task_log.log:
            if log.tier == mdl.Tier.error:
                return True

    if task_log.phases:
        for _, phase in task_log.phases.items():
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


async def create_alert(id: str, category: str):
    alert, _ = await ttm.Alert.update_or_create(
        {
            "original_id": id,
            "category": category,
            "unix_millis_created_time": round(datetime.now().timestamp() * 1e3),
            "acknowledged_by": None,
            "unix_millis_acknowledged_time": None,
        },
        id=id,
    )
    alert_pydantic = await ttm.AlertPydantic.from_tortoise_orm(alert)
    return alert_pydantic


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
            alert = await create_alert(task_state.booking.id, "task")
            alert_events.alerts.on_next(alert)

    elif payload_type == "task_log_update":
        task_log = mdl.TaskEventLog(**msg["data"])
        await task_repo.save_task_log(task_log)
        task_events.task_event_logs.on_next(task_log)

        if task_log_has_error(task_log):
            alert = await create_alert(task_log.task_id, "task")
            alert_events.alerts.on_next(alert)

    elif payload_type == "fleet_state_update":
        fleet_state = mdl.FleetState(**msg["data"])
        await fleet_repo.save_fleet_state(fleet_state)
        fleet_events.fleet_states.on_next(fleet_state)

        for name, state in fleet_state.robots.items():
            # Alert ID is the fleet name and robot name delimited by two
            # underscores. If this is modified, be sure to change how it is
            # parsed in the RobotAlertHandler dashboard component
            alert_id = f"{fleet_state.name}__{name}"
            alert_exists = await ttm.Alert.exists(id=alert_id)

            # If the robot state is an error and the alert does not exist yet,
            # we create a new alert and pass it on as an event
            if state.status == mdl.Status2.error and not alert_exists:
                alert = await create_alert(alert_id, "robot")
                alert_events.alerts.on_next(alert)
            # If there is an existing alert and the robot status is not error,
            # we consider it to have resolved itself, we create a new alert with
            # id containing the current unix millis, set it to acknowledged,
            # delete the old alert, pass the acknowledged alert as an event so
            # the frontend can close any open dialogs
            elif state.status != mdl.Status2.error and alert_exists:
                alert = await ttm.Alert.get_or_none(id=alert_id)
                if alert is not None:
                    ack_time = datetime.now()
                    epoch = datetime.utcfromtimestamp(0)
                    ack_unix_millis = round((ack_time - epoch).total_seconds() * 1000)
                    new_id = f"{alert_id}__{ack_unix_millis}"

                    ack_alert = alert.clone(pk=new_id)
                    # TODO(aaronchongth): remove the following line once we bump
                    # tortoise-orm to include
                    # https://github.com/tortoise/tortoise-orm/pull/1131. This
                    # is a temporary workaround.
                    ack_alert._custom_generated_pk = True
                    ack_alert.update_from_dict(
                        {
                            "acknowledged_by": name,
                            "unix_millis_acknowledged_time": round(
                                ack_time.timestamp() * 1e3
                            ),
                        }
                    )
                    await ack_alert.save()
                    await alert.delete()
                    ack_alert_pydantic = await ttm.AlertPydantic.from_tortoise_orm(
                        ack_alert
                    )
                    alert_events.alerts.on_next(ack_alert_pydantic)

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
        pass
