from reactivex.subject import BehaviorSubject, Subject

from api_server import models as mdl
from api_server.fast_io import singleton_dep


class RmfEvents:
    def __init__(self):
        self.door_states = Subject[mdl.DoorState]()
        self.lift_states = Subject[mdl.LiftState]()
        self.dispenser_states = Subject[mdl.DispenserState]()
        self.ingestor_states = Subject[mdl.IngestorState]()
        self.fleet_states = Subject[mdl.FleetState]()
        self.building_map = BehaviorSubject[mdl.BuildingMap | None](None)
        self.beacons = Subject[mdl.BeaconState]()
        self.delivery_alerts = Subject[mdl.DeliveryAlert]()
        self.fire_alarm_trigger = BehaviorSubject[mdl.FireAlarmTriggerState | None](
            None
        )


@singleton_dep
def get_rmf_events():
    return RmfEvents()


class TaskEvents:
    def __init__(self):
        self.task_states = Subject[mdl.TaskState]()
        self.task_event_logs = Subject[mdl.TaskEventLog]()


@singleton_dep
def get_task_events():
    return TaskEvents()


class FleetEvents:
    def __init__(self):
        self.fleet_states = Subject[mdl.FleetState]()
        self.fleet_logs = Subject[mdl.FleetLog]()


@singleton_dep
def get_fleet_events():
    return FleetEvents()


class AlertEvents:
    def __init__(self):
        self.alert_requests = Subject()
        self.alert_responses = Subject()


@singleton_dep
def get_alert_events():
    return AlertEvents()


class BeaconEvents:
    def __init__(self):
        self.beacons = Subject()


@singleton_dep
def get_beacon_events():
    return BeaconEvents()


class RioEvents:
    def __init__(self):
        self.rios = Subject[mdl.Rio]()


@singleton_dep
def get_rio_events():
    return RioEvents()
