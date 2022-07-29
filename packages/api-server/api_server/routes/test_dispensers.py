from typing import List
from uuid import uuid4

from api_server.rmf_io import rmf_events
from api_server.test import AppFixture, make_dispenser_state


class TestDispensersRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.dispenser_states = [make_dispenser_state(f"test_{uuid4()}")]

        for x in cls.dispenser_states:
            rmf_events.dispenser_states.on_next(x)

    def test_get_dispensers(self):
        resp = self.client.get("/dispensers")
        self.assertEqual(resp.status_code, 200)
        results: List = resp.json()
        self.assertIsNotNone(
            next(
                (x for x in results if x["guid"] == self.dispenser_states[0].guid), None
            )
        )

    def test_get_dispenser_state(self):
        resp = self.client.get(f"/dispensers/{self.dispenser_states[0].guid}/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual(self.dispenser_states[0].guid, state["guid"])

    def test_sub_dispenser_state(self):
        msg = next(
            self.subscribe_sio(f"/dispensers/{self.dispenser_states[0].guid}/state")
        )
        self.assertEqual(self.dispenser_states[0].guid, msg.guid)  # type: ignore
