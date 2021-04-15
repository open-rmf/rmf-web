from models.dispenser_state import DispenserState
from models.door_state import DoorState
from models.fleet_state import FleetState
from models.health import HealthStatus
from models.ingestor_state import IngestorState
from models.lift_state import LiftState
from parsers.dispenser_state_parser import dispenser_state_parser
from parsers.doors_state_parser import doors_state_parser
from parsers.fleet_state_parser import fleet_state_parser
from parsers.health_parser import health_status_parser
from parsers.ingestor_state_parser import ingestor_state_parser
from parsers.lift_state_parser import lift_state_parser


# This function dispatchs to the correct handler dependending on the text content.
async def log_model_dispacher(fullstring: str):
    if "dispenser states" in fullstring.lower():
        DispenserState.create(dispenser_state_parser(fullstring))

    elif "door_state" in fullstring.lower():
        DoorState.create(doors_state_parser(fullstring))

    elif "fleet_state" in fullstring.lower():
        FleetState.create(fleet_state_parser(fullstring))

    elif "lift_state" in fullstring.lower():
        LiftState.create(lift_state_parser(fullstring))

    elif "ingestor_state" in fullstring.lower():
        IngestorState.create(lift_state_parser(fullstring))

    elif "tasks" in fullstring.lower():
        print("not implemented")

    # Health
    elif "dispenser_health" in fullstring.lower():
        HealthStatus.create(health_status_parser(fullstring, "dispenser_health"))

    elif "door_health" in fullstring.lower():
        HealthStatus.create(health_status_parser(fullstring, "door_health"))

    elif "ingestor_health" in fullstring.lower():
        HealthStatus.create(health_status_parser(fullstring, "ingestor_health"))

    elif "lift_health" in fullstring.lower():
        HealthStatus.create(health_status_parser(fullstring, "lift_health"))

    elif "robot health" in fullstring.lower():
        HealthStatus.create(health_status_parser(fullstring, "robot health"))
