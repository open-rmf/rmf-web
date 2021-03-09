from rx.subject import Subject


class RmfGateway:
    def __init__(self):
        # NOTE: the rx type hints don't actually work https://github.com/ReactiveX/RxPY/issues/514
        self.door_states = Subject()  # Subject[DoorState]
        self.door_health = Subject()  # Subject[DoorHealth]
        self.lift_states = Subject()  # Subject[LiftState]
        self.lift_health = Subject()  # Subject[LiftHealth]
        self.dispenser_states = Subject()  # Subject[DispenserState]
        self.building_map = Subject()  # Subject[BuildingMap]
