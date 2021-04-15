import json

from models.door_state import DoorState

from .log_type_parser import get_log_type

# {'log': 'INFO:app.BookKeeper.door_state:{"door_time": {"sec": 1596, "nanosec": 548000000}, "door_name": "hardware_door", "current_mode": {"value": 0}}\n', 'stream': 'stdout'}


async def doors_state_parser(fullstring: str):
    print(fullstring)
    modified_string = fullstring.replace("door_state:", "")
    door_state_json = json.loads(modified_string)

    return {
        "state": DoorState.service.get_state_name(
            door_state_json["current_mode"]["value"]
        ),
        "payload": modified_string,
        "name": door_state_json["door_name"],
    }
