# NOTE: This will eventually replace `gateway.py``
import sys
from typing import Any, Dict

from fastapi import FastAPI, WebSocket, WebSocketDisconnect

from . import models as mdl
from .models import tortoise_models as dbmdl
from .rmf_io import fleet_events, task_events

app = FastAPI()


# FIXME: Log updates are very inefficient because we are re-writing the entire logs
# on every update. More efficient solutions will require proper relational schemas.
async def process_msg(msg: Dict[str, Any]) -> None:
    payload_type: str = msg["type"]
    if not isinstance(payload_type, str):
        print("'type' must be a string", file=sys.stderr)

    if payload_type == "task_state_update":
        task_state = mdl.TaskState.construct(**msg["data"])
        await task_state.save()
        task_events.task_states.on_next(task_state)
    elif payload_type == "task_log_update":
        task_log = mdl.TaskEventLog.construct(**msg["data"])
        current_log = mdl.TaskEventLog.from_db(
            (
                await dbmdl.TaskEventLog.get_or_create(
                    {"data": task_log.json()}, task_id=task_log.task_id
                )
            )[0]
        )
        if task_log.log:
            if current_log.log is None:
                current_log.log = []
            current_log.log.extend(task_log.log)
        if task_log.phases:
            if current_log.phases is None:
                current_log.phases = {}
            for phase_name, phase in task_log.phases.items():
                current_phase = current_log.phases.setdefault(phase_name, {})
                if "log" in phase:
                    current_phase_logs = current_phase.setdefault("log", [])
                    current_phase_logs.extend(phase["log"])
                if "events" in phase:
                    current_events = current_phase.setdefault("events", {})
                    for event_name, event in phase["events"].items():
                        current_event = current_events.setdefault(event_name, [])
                        current_event.extend(event)
        await current_log.save()
        task_events.task_event_logs.on_next(task_log)
    elif payload_type == "fleet_state_update":
        fleet_state = mdl.FleetState.construct(**msg["data"])
        await fleet_state.save()
        fleet_events.fleet_states.on_next(fleet_state)
    elif payload_type == "fleet_log_update":
        fleet_log = mdl.FleetLog.construct(**msg["data"])
        current_log = mdl.FleetLog.from_db(
            (
                await dbmdl.FleetLog.get_or_create(
                    {"data": fleet_log.json()}, name=fleet_log.name
                )
            )[0]
        )
        if fleet_log.log:
            if current_log.log is None:
                current_log.log = []
            current_log.log.extend(fleet_log.log)
        if fleet_log.robots:
            if current_log.robots is None:
                current_log.robots = {}
            for robot_name, robot in fleet_log.robots.items():
                current_robot = current_log.robots.setdefault(robot_name, [])
                current_robot.extend(robot)
        await current_log.save()
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
