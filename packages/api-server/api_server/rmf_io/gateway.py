from rx.subject import Subject


class RmfGateway():
    def __init__(self):
        self.door_states = Subject()
