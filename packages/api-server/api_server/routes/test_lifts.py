from rmf_lift_msgs.msg import LiftRequest

from .test_fixtures import RouteFixture


class TestLiftsRoute(RouteFixture):
    def test_lift_request(self):
        fut = self.subscribe_one(LiftRequest, "adapter_lift_requests")
        resp = self.client.post(
            "/lifts/test_lift/request",
            json={
                "request_type": LiftRequest.REQUEST_AGV_MODE,
                "door_mode": LiftRequest.DOOR_OPEN,
                "destination": "L1",
            },
        )
        self.assertEqual(resp.status_code, 200)
        fut.result(1)
