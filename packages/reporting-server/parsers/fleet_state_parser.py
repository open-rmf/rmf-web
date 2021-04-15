import json

from models.door_state import DoorState

from .log_type_parser import get_log_type

# {"name": "tinyRobot", "robots": [{"name": "tinyRobot1", "model": "", "task_id": "", "seq": 3194, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1600, "nanosec": 189000000}, "x": 11.55367374420166, "y": -11.317498207092285, "yaw": -1.5998055934906006, "level_name": "L1", "index": 0}, "path": []}, {"name": "tinyRobot2", "model": "", "task_id": "", "seq": 3194, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1600, "nanosec": 189000000}, "x": 15.15751838684082, "y": -11.22861385345459, "yaw": -1.5839799642562866, "level_name": "L1", "index": 0}, "path": []}]}\n', 'stream': 'stdout'}


async def fleet_state_parser(fullstring: str):
    print(fullstring)
    modified_string = fullstring.replace("fleet_state:", "")
    state_json = json.loads(modified_string)

    return {
        "payload": modified_string,
        "name": state_json["name"],
        "robots": state_json["robots"],
    }
