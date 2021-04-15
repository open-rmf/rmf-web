import json

from models.door_state import DoorState
from models.lift_state import LiftState

from .log_type_parser import get_log_type


async def lift_state_parser(fullstring: str):
    print(fullstring)
    modified_string = fullstring.replace("lift_state:", "")
    state_json = json.loads(modified_string)

    return {
        "state": LiftState.service.get_state_name(state_json["current_mode"]["value"]),
        "payload": modified_string,
        "motion_state": LiftState.service.get_motion_state_name(
            state_json["motion_state"]["value"]
        ),
        "door_state": DoorState.service.get_state_name(
            state_json["door_state"]["value"]
        ),
        "session_id": state_json["session_id"]["value"],
        "destination_floor": state_json["destination_floor"],
        "current_floor": state_json["current_floor"],
    }
