import json

from models.health import HealthStatus

# {'log': 'INFO:app.BookKeeper.door_health:{"id": "hardware_door", "health_status": "HealthStatus.HEALTHY", "health_message": null}\n', 'stream': 'stdout'}


async def health_status_parser(fullstring: str, health_device: str):
    modified_string = fullstring.replace(health_device + ":", "")
    state_json = json.loads(modified_string)

    return {
        "device": health_device,
        "actor_id": state_json["id"],
        "payload": modified_string,
        "health_status": state_json["health_status"],
        "health_message": state_json["health_message"],
    }
