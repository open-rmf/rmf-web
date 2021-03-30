from rmf_door_msgs.msg import DoorMode, DoorRequest

from .test_fixtures import RouteFixture


class TestDoorsRoute(RouteFixture):
    def test_door_request(self):
        fut = self.subscribe_one(DoorRequest, "adapter_door_requests")
        resp = self.client.post(
            "/doors/test_door/request", json={"mode": DoorMode.MODE_OPEN}
        )
        self.assertEqual(resp.status_code, 200)
        fut.result(1)
