from rmf_door_msgs.msg import DoorMode as RmfDoorMode
from rmf_door_msgs.msg import DoorRequest as RmfDoorRequest
from rmf_door_msgs.msg import DoorState as RmfDoorState

from .test_fixtures import RouteFixture, try_until


class TestDoorsRoute(RouteFixture):
    def test_door_state(self):
        pub = self.node.create_publisher(RmfDoorState, "door_states", 10)
        rmf_door_state = RmfDoorState(door_name="test_door")

        def try_get():
            pub.publish(rmf_door_state)
            return self.session.get(f"{self.base_url}/doors/test_door/state")

        resp = try_until(
            try_get,
            lambda x: x.status_code == 200,
        )
        self.assertEqual(resp.status_code, 200)

    def test_door_request(self):
        fut = self.subscribe_one(RmfDoorRequest, "adapter_door_requests")
        resp = self.session.post(
            f"{self.base_url}/doors/test_door/request",
            json={"mode": RmfDoorMode.MODE_OPEN},
        )
        self.assertEqual(resp.status_code, 200)
        door_request: RmfDoorRequest = self.spin_until(fut, 3)
        self.assertEqual(door_request.door_name, "test_door")
        self.assertEqual(door_request.requested_mode.value, RmfDoorMode.MODE_OPEN)
