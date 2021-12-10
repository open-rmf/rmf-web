from uuid import uuid4

from rmf_lift_msgs.msg import LiftRequest as RmfLiftRequest

from api_server.test import AppFixture, make_building_map, make_lift_state, try_until


class TestLiftsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.building_map = make_building_map()
        cls.lift_states = [make_lift_state(f"test_{uuid4()}")]

        async def prepare_db():
            await cls.building_map.save()
            for x in cls.lift_states:
                await x.save()

        cls.run_in_app_loop(prepare_db())

    def test_get_lifts(self):
        resp = self.session.get("/lifts")
        self.assertEqual(200, resp.status_code)
        lifts = resp.json()
        self.assertEqual(1, len(lifts))
        self.assertEqual("test_lift", lifts[0]["name"])

    def test_get_lift_state(self):
        resp = self.session.get(f"/lifts/{self.lift_states[0].lift_name}/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual(self.lift_states[0].lift_name, state["lift_name"])

    def test_sub_lift_state(self):
        fut = self.subscribe_sio(f"/lifts/{self.lift_states[0].lift_name}/state")
        try_until(fut.done, lambda x: x)
        result = fut.result(0)
        self.assertEqual(self.lift_states[0].lift_name, result["lift_name"])

    def test_request_lift(self):
        resp = self.session.post(
            "/lifts/test_lift/request",
            json={
                "request_type": RmfLiftRequest.REQUEST_AGV_MODE,
                "door_mode": RmfLiftRequest.DOOR_OPEN,
                "destination": "L1",
            },
        )
        self.assertEqual(resp.status_code, 200)
