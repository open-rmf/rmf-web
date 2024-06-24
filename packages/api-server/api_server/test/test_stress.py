import unittest
from uuid import uuid4

from api_server.rmf_io import rmf_events
from api_server.test.test_data import make_door_state
from api_server.test.test_fixtures import AppFixture


@unittest.skip("manual test")
class TestStress(AppFixture):
    def test_stress(self):
        """
        Continuously make socketio connections to check for memory leaks.
        This test is not automated, user should monitor their system memory while
        this is running.
        """
        door_state = make_door_state(f"test_{uuid4()}")
        rmf_events.door_states.on_next(door_state)

        while True:
            with self.subscribe_sio(f"/doors/{door_state.door_name}/state") as sub:
                next(sub)
