import json

from models.ingestor_state import IngestorState

# {'log': 'INFO:app.BookKeeper.dispenser_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n', 'stream': 'stdout'}


async def ingestor_state_parser(fullstring: str):
    print(fullstring)
    modified_string = fullstring.replace("ingestor_state:", "")
    state_json = json.loads(modified_string)

    return {
        "state": IngestorState.service.get_state_name(state_json["mode"]),
        "payload": modified_string,
        "guid": state_json["guid"],
    }
