from typing import Any, Callable, Dict

from rmf_dispenser_msgs.msg import DispenserState
from rmf_door_msgs.msg import DoorState
from rmf_fleet_msgs.msg import FleetState
from rmf_lift_msgs.msg import LiftState
from rx import Observable
from rx.subject import BehaviorSubject, Subject

from ..models import DispenserHealth, DoorHealth, LiftHealth, RobotHealth


class RmfGateway:
    def __init__(
        self,
    ):
        # NOTE: the rx type hints don't actually work https://github.com/ReactiveX/RxPY/issues/514
        self.door_states = Subject()  # Subject[DoorState]
        self.current_door_states: Dict[str, DoorState] = {}
        self._save_event(
            self.door_states,
            self.current_door_states,
            lambda x: x.door_name,
        )

        self.door_health = Subject()  # Subject[DoorHealth]
        self.current_door_health: Dict[str, DoorHealth] = {}
        self._save_event(
            self.door_health,
            self.current_door_health,
            lambda x: x.id_,
        )

        self.lift_states = Subject()  # Subject[LiftState]
        self.current_lift_states: Dict[str, LiftState] = {}
        self._save_event(
            self.lift_states,
            self.current_lift_states,
            lambda x: x.lift_name,
        )

        self.lift_health = Subject()  # Subject[LiftHealth]
        self.current_lift_health: Dict[str, LiftHealth] = {}
        self._save_event(
            self.lift_health,
            self.current_lift_health,
            lambda x: x.id_,
        )

        self.dispenser_states = Subject()  # Subject[DispenserState]
        self.current_dispenser_states: Dict[str, DispenserState] = {}
        self._save_event(
            self.dispenser_states,
            self.current_dispenser_states,
            lambda x: x.guid,
        )

        self.dispenser_health = Subject()  # Subject[DispenserHealth]
        self.current_dispenser_health: Dict[str, DispenserHealth] = {}
        self._save_event(
            self.dispenser_health,
            self.current_dispenser_health,
            lambda x: x.id_,
        )

        self.fleet_states = Subject()  # Subject[FleetState]
        self.current_fleet_states: Dict[str, FleetState] = {}
        self._save_event(
            self.fleet_states,
            self.current_fleet_states,
            lambda x: x.name,
        )

        self.robot_health = Subject()  # Subject[RobotHealth]
        self.current_robot_health: Dict[str, RobotHealth] = {}
        self._save_event(
            self.robot_health,
            self.current_robot_health,
            lambda x: x.id_,
        )

        self.building_map = BehaviorSubject(  # BehaviorSubject[Optional[BuildingMap]]
            None
        )

    @staticmethod
    def _save_event(source: Observable, dic, key_mapper: Callable[[Any], str]):
        def on_next(data):
            dic[key_mapper(data)] = data

        source.subscribe(on_next)
