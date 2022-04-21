# NOTE: This will eventually replace `gateway.py``
from typing import Any, Dict

from fastapi import FastAPI, WebSocket, WebSocketDisconnect

from . import models as mdl
from .logger import logger as base_logger
from .repositories import FleetRepository, TaskRepository
from .rmf_io import fleet_events, task_events

app = FastAPI()
logger = base_logger.getChild("RmfGatewayApp")
user: mdl.User = mdl.User(username="_rmf_gateway_app", is_admin=True)
task_repo = TaskRepository(user)
fleet_repo = FleetRepository(user)


async def process_msg(msg: Dict[str, Any]) -> None:
    payload_type: str = msg["type"]
    if not isinstance(payload_type, str):
        logger.error("error processing message, 'type' must be a string")
        return
    logger.debug(msg)

    if payload_type == "task_state_update":
        task_state = mdl.TaskState(**msg["data"])
        await task_repo.save_task_state(task_state)
        task_events.task_states.on_next(task_state)
    elif payload_type == "task_log_update":
        task_log = mdl.TaskEventLog(**msg["data"])
        await task_repo.save_task_log(task_log)
        task_events.task_event_logs.on_next(task_log)
    elif payload_type == "fleet_state_update":
        fleet_state = mdl.FleetState(**msg["data"])
        await fleet_state.save()
        fleet_events.fleet_states.on_next(fleet_state)
    elif payload_type == "fleet_log_update":
        fleet_log = mdl.FleetLog(**msg["data"])
        await fleet_repo.save_fleet_log(fleet_log)
        fleet_events.fleet_logs.on_next(fleet_log)


@app.websocket("/")
async def rmf_gateway(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            msg: Dict[str, Any] = await websocket.receive_json()
            await process_msg(msg)
    except WebSocketDisconnect:
        pass
