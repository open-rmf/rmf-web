from reactivex.subject import BehaviorSubject, Subject

from api_server import models as mdl


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
        self.alerts = Subject()


alert_events = AlertEvents()


class BeaconEvents:
    def __init__(self):
        self.beacons = Subject()


beacon_events = BeaconEvents()
