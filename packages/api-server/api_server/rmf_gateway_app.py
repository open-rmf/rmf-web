# NOTE: This will eventually replace `gateway.py``
import sys
from typing import Any, Dict

from fastapi import FastAPI, WebSocket

from .models import TaskState
from .rmf_io import RmfEvents

app = FastAPI()
rmf_events = RmfEvents.singleton()


@app.websocket("/")
async def rmf_gateway(websocket: WebSocket):
    await websocket.accept()
    while True:
        data: Dict[str, Any] = await websocket.receive_json()
        payload_type: str = data["type"]
        if not isinstance(payload_type, str):
            print("'type' must be a string", file=sys.stderr)

        if payload_type == "task_state_update":
            task_state = TaskState.construct(**data)
            rmf_events.task_states.on_next(task_state)
