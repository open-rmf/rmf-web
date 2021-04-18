import json

from models.door_state import DoorState

from .log_type_parser import get_log_type


async def fleet_state_parser(fullstring: str):
    modified_string = fullstring.replace("fleet_state:", "")
    state_json = json.loads(modified_string)

    return {
        "payload": modified_string,
        "name": state_json["name"],
        "robots": state_json["robots"],
    }
