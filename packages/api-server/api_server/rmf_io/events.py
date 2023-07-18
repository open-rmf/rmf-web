from rx.subject.behaviorsubject import BehaviorSubject
from rx.subject.subject import Subject


class RmfEvents:
    def __init__(self):
        # NOTE: the rx type hints don't actually work https://github.com/ReactiveX/RxPY/issues/514
        self.door_states = Subject()  # DoorState
        self.door_health = Subject()  # DoorHealth
        self.lift_states = Subject()  # LiftState
        self.lift_health = Subject()  # LiftHealth
        self.dispenser_states = Subject()  # DispenserState
        self.dispenser_health = Subject()  # DispenserHealth
        self.ingestor_states = Subject()  # IngestorState
        self.ingestor_health = Subject()  # IngestorHealth
        self.fleet_states = Subject()  # FleetState
        self.robot_health = Subject()  # RobotHealth
        self.building_map = BehaviorSubject(None)  # Optional[BuildingMap]


rmf_events = RmfEvents()


class TaskEvents:
    def __init__(self):
        self.task_states = Subject()  # TaskState
        self.task_event_logs = Subject()  # TaskEventLog


task_events = TaskEvents()


class FleetEvents:
    def __init__(self):
        self.fleet_states = Subject()  # FleetState
        self.fleet_logs = Subject()  # FleetLog


fleet_events = FleetEvents()


class AlertEvents:
    def __init__(self):
        self.alerts = Subject()  # Alert


alert_events = AlertEvents()


class BeaconEvents:
    def __init__(self):
        self.beacons = Subject()  # Beacons


beacon_events = BeaconEvents()
