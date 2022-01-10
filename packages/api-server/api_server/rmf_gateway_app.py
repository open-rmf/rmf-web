# NOTE: This will eventually replace `gateway.py``
from typing import Any, Dict

from fastapi import FastAPI, WebSocket, WebSocketDisconnect

from . import models as mdl
from .logger import logger as base_logger
from .rmf_io import fleet_events, task_events

app = FastAPI()
logger = base_logger.getChild("RmfGatewayApp")


async def process_msg(msg: Dict[str, Any]) -> None:
    payload_type: str = msg["type"]
    if not isinstance(payload_type, str):
        logger.error("'type' must be a string")
    logger.info(f"received message of type '{payload_type}'")

    if payload_type == "task_state_update":
        task_state = mdl.TaskState(**msg["data"])
        await task_state.save()
        task_events.task_states.on_next(task_state)
    elif payload_type == "task_log_update":
        task_log = mdl.TaskEventLog(**msg["data"])
        await task_log.save()
        task_events.task_event_logs.on_next(task_log)
    elif payload_type == "fleet_state_update":
        fleet_state = mdl.FleetState(**msg["data"])
        await fleet_state.save()
        fleet_events.fleet_states.on_next(fleet_state)
    elif payload_type == "fleet_log_update":
        fleet_log = mdl.FleetLog(**msg["data"])
        await fleet_log.save()
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
