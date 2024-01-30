from reactivex.subject import BehaviorSubject, Subject
from tortoise.contrib.pydantic.base import PydanticModel

from api_server import models as mdl


class RmfEvents:
    def __init__(self):
        self.door_states = Subject[mdl.DoorState]()
        self.door_health = Subject[mdl.DoorHealth]()
        self.lift_states = Subject[mdl.LiftState]()
        self.lift_health = Subject[mdl.LiftHealth]()
        self.dispenser_states = Subject[mdl.DispenserState]()
        self.dispenser_health = Subject[mdl.DispenserHealth]()
        self.ingestor_states = Subject[mdl.IngestorState]()
        self.ingestor_health = Subject[mdl.IngestorHealth]()
        self.fleet_states = Subject[mdl.FleetState]()
        self.robot_health = Subject[mdl.RobotHealth]()
        self.building_map = BehaviorSubject[mdl.BuildingMap | None](None)


rmf_events = RmfEvents()


class TaskEvents:
    def __init__(self):
        self.task_states = Subject[mdl.TaskState]()
        self.task_event_logs = Subject[mdl.TaskEventLog]()


task_events = TaskEvents()


class FleetEvents:
    def __init__(self):
        self.fleet_states = Subject[mdl.FleetState]()
        self.fleet_logs = Subject[mdl.FleetLog]()


fleet_events = FleetEvents()


class AlertEvents:
    def __init__(self):
        self.alerts = Subject[PydanticModel]()


alert_events = AlertEvents()


class BeaconEvents:
    def __init__(self):
        self.beacons = Subject[PydanticModel]()


beacon_events = BeaconEvents()
