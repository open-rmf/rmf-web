from reactivex.subject import BehaviorSubject, Subject

from api_server import models as mdl
from api_server.fast_io import SingletonDep


class RmfEvents(SingletonDep):
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

    async def __aexit__(self, exc_type, exc_value, traceback):
        pass


class TaskEvents(SingletonDep):
    def __init__(self):
        self.task_states = Subject[mdl.TaskState]()
        self.task_event_logs = Subject[mdl.TaskEventLog]()

    async def __aexit__(self, exc_type, exc_value, traceback):
        pass


class FleetEvents(SingletonDep):
    def __init__(self):
        self.fleet_states = Subject[mdl.FleetState]()
        self.fleet_logs = Subject[mdl.FleetLog]()

    async def __aexit__(self, exc_type, exc_value, traceback):
        pass


class AlertEvents(SingletonDep):
    def __init__(self):
        self.alerts = Subject()

    async def __aexit__(self, exc_type, exc_value, traceback):
        pass


class BeaconEvents(SingletonDep):
    def __init__(self):
        self.beacons = Subject()

    async def __aexit__(self, exc_type, exc_value, traceback):
        pass
