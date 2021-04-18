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
    print(fullstring)
    if "dispenser_state:" in fullstring.lower():
        data = await dispenser_state_parser(fullstring)
        await DispenserState.create(**data)

    elif "door_state:" in fullstring.lower():
        data = await doors_state_parser(fullstring)
        await DoorState.create(**data)

    elif "fleet_state:" in fullstring.lower():
        data = await fleet_state_parser(fullstring)
        await FleetState.create(**data)

    elif "lift_state:" in fullstring.lower():
        data = await lift_state_parser(fullstring)
        await LiftState.create(**data)

    elif "ingestor_state:" in fullstring.lower():
        data = await ingestor_state_parser(fullstring)
        await IngestorState.create(**data)

    elif "tasks:" in fullstring.lower():
        print("not implemented")

    # Health
    elif "dispenser_health:" in fullstring.lower():
        data = await health_status_parser(fullstring, "dispenser_health")
        await HealthStatus.create(**data)

    elif "door_health:" in fullstring.lower():
        data = await health_status_parser(fullstring, "door_health")
        await HealthStatus.create(**data)

    elif "ingestor_health:" in fullstring.lower():
        data = await health_status_parser(fullstring, "ingestor_health")
        await HealthStatus.create(**data)

    elif "lift_health:" in fullstring.lower():
        data = await health_status_parser(fullstring, "lift_health")
        await HealthStatus.create(**data)

    elif "robot_health:" in fullstring.lower():
        data = await health_status_parser(fullstring, "robot_health")
        await HealthStatus.create(**data)
