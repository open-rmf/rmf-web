import json

from models.ingestor_state import IngestorState


async def ingestor_state_parser(fullstring: str):
    modified_string = fullstring.replace("ingestor_state:", "")
    state_json = json.loads(modified_string)

    return {
        "state": IngestorState.service.get_state_name(state_json["mode"]),
        "payload": modified_string,
        "guid": state_json["guid"],
    }
