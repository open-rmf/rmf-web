from uuid import uuid4

from rmf_lift_msgs.msg import LiftRequest as RmfLiftRequest

from api_server.models import LiftState
from api_server.test import AppFixture, make_building_map, make_lift_state


class TestLiftsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.building_map = make_building_map()
        portal = cls.get_portal()
        portal.call(cls.building_map.save)

        cls.lift_states = [make_lift_state(f"test_{uuid4()}")]
        for x in cls.lift_states:
            portal.call(x.save)

    def test_get_lifts(self):
        resp = self.client.get("/lifts")
        self.assertEqual(200, resp.status_code)
        lifts = resp.json()
        self.assertEqual(1, len(lifts))
        self.assertEqual("test_lift", lifts[0]["name"])

    def test_get_lift_state(self):
        resp = self.client.get(f"/lifts/{self.lift_states[0].lift_name}/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual(self.lift_states[0].lift_name, state["lift_name"])

    def test_sub_lift_state(self):
        with self.subscribe_sio(f"/lifts/{self.lift_states[0].lift_name}/state") as sub:
            msg = LiftState(**next(sub))
            self.assertEqual(self.lift_states[0].lift_name, msg.lift_name)

    def test_request_lift(self):
        resp = self.client.post(
            "/lifts/test_lift/request",
            json={
                "request_type": RmfLiftRequest.REQUEST_AGV_MODE,
                "door_mode": RmfLiftRequest.DOOR_OPEN,
                "destination": "L1",
            },
        )
        self.assertEqual(resp.status_code, 200)
