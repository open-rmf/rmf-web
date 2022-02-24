from typing import List
from uuid import uuid4

from api_server.test import AppFixture, make_dispenser_state


class TestDispensersRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.dispenser_states = [make_dispenser_state(f"test_{uuid4()}")]

        async def prepare_db():
            for x in cls.dispenser_states:
                await x.save()

        cls.run_in_app_loop(prepare_db())

    def test_get_dispensers(self):
        resp = self.session.get("/dispensers")
        self.assertEqual(resp.status_code, 200)
        results: List = resp.json()
        self.assertIsNotNone(
            next(
                (x for x in results if x["guid"] == self.dispenser_states[0].guid), None
            )
        )

    def test_get_dispenser_state(self):
        resp = self.session.get(f"/dispensers/{self.dispenser_states[0].guid}/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual(self.dispenser_states[0].guid, state["guid"])

    def test_sub_dispenser_state(self):
        fut = self.subscribe_sio(f"/dispensers/{self.dispenser_states[0].guid}/state")
        result = fut.result(1)
        self.assertEqual(self.dispenser_states[0].guid, result["guid"])
