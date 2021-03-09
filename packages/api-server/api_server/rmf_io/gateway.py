from typing import Any, Callable, Dict

from rmf_door_msgs.msg import DoorState
from rmf_lift_msgs.msg import LiftState
from rx import Observable
from rx.subject import BehaviorSubject, Subject

from ..models import DoorHealth, LiftHealth


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
            lambda x: x.name,
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
            lambda x: x.name,
        )

        self.building_map = BehaviorSubject(
            None
        )  # BehaviorSubject[Optional[BuildingMap]]

    @staticmethod
    def _save_event(source: Observable, dic, key_mapper: Callable[[Any], str]):
        def on_next(data):
            dic[key_mapper(data)] = data

        source.subscribe(on_next)
