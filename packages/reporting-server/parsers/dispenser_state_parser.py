import json

from models.dispenser_state import DispenserState

# {'log': 'INFO:app.BookKeeper.dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n', 'stream': 'stdout'}


async def dispenser_state_parser(fullstring: str):
    modified_string = fullstring.replace("dispenser_state:", "")
    state_json = json.loads(modified_string)

    return {
        "state": DispenserState.service.get_state_name(state_json["mode"]),
        "payload": modified_string,
        "guid": state_json["guid"],
    }
