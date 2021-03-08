from typing import Dict, Optional

from building_map_msgs.msg import BuildingMap
from rmf_door_msgs.msg import DoorState
from rmf_lift_msgs.msg import LiftState
from rx.subject import BehaviorSubject, Subject

from ..models import DoorHealth, LiftHealth


class RmfGateway:
    def __init__(
        self,
    ):
        # NOTE: the rx type hints don't actually work https://github.com/ReactiveX/RxPY/issues/514
        self.door_states: Subject[DoorState] = Subject()
        self.current_door_states: Dict[str, DoorState] = {}
        self._init_door_states()

        self.door_health: Subject[DoorHealth] = Subject()
        self.current_door_health: Dict[str, DoorHealth] = {}
        self._init_door_health()

        self.building_map: BehaviorSubject[Optional[BuildingMap]] = BehaviorSubject(
            None
        )

        self.lift_states: Subject[LiftState] = Subject()
        self.current_lift_states: Dict[str, LiftState] = {}
        self._init_lift_state()

        self.lift_health: Subject[LiftHealth] = Subject()
        self.current_lift_health: Dict[str, LiftHealth] = {}
        self._init_lift_health()

    def _init_door_states(self):
        def on_next(state: DoorState):
            self.current_door_states[state.door_name] = state

        self.door_states.subscribe(on_next)

    def _init_door_health(self):
        def on_next(health: DoorHealth):
            self.current_door_health[health.name] = health

        self.door_health.subscribe(on_next)

    def _init_lift_state(self):
        def on_next(state: LiftState):
            self.current_lift_states[state.lift_name] = state

        self.lift_states.subscribe(on_next)

    def _init_lift_health(self):
        def on_next(health: LiftHealth):
            self.current_lift_health[health.name] = health

        self.door_health.subscribe(on_next)
