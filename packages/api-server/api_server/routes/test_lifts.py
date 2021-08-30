from unittest.mock import Mock

from api_server.test import AppFixture, make_building_map, make_lift_state, try_until
from rmf_lift_msgs.msg import LiftRequest as RmfLiftRequest


class TestLiftsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.app.rmf_events().building_map.on_next(make_building_map())

    def test_get_lifts(self):
        resp = self.session.get("/lifts")
        self.assertEqual(200, resp.status_code)
        lifts = resp.json()
        self.assertEqual(1, len(lifts))
        self.assertEqual("test_lift", lifts[0]["name"])

    def test_get_lift_state(self):
        self.app.rmf_events().lift_states.on_next(make_lift_state())
        resp = self.session.get("/lifts/test_lift/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual("test_lift", state["lift_name"])

    def test_watch_lift_state(self):
        lift_state = make_lift_state()
        lift_state.lift_time.sec = 1
        fut = self.subscribe_sio("/lifts/test_lift/state")

        def wait():
            self.app.rmf_events().lift_states.on_next(lift_state)
            return fut.result(0)

        result = try_until(wait, lambda _: True)
        self.assertEqual(1, result["lift_time"]["sec"])

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
        mock = self.app.rmf_gateway().request_lift
        mock: Mock
        self.assertEqual(mock.call_args.args[0], "test_lift")
        self.assertEqual(mock.call_args.args[1], "L1")
        self.assertEqual(mock.call_args.args[2], RmfLiftRequest.REQUEST_AGV_MODE)
        self.assertEqual(mock.call_args.args[3], RmfLiftRequest.DOOR_OPEN)
