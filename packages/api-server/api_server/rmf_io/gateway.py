from rx.subject import BehaviorSubject, Subject

from rmf_door_msgs.msg import DoorState


class RmfGateway():
    def __init__(self):
        self.door_states = Subject()
        self._current_door_states: dict[str, DoorState] = {}
        self.building_map = BehaviorSubject(None)

        self.door_states.subscribe(self._update_door_states)

    @property
    def current_door_states(self):
        return self._current_door_states

    def _update_door_states(self, state: DoorState):
        self._current_door_states[state.door_name] = state
