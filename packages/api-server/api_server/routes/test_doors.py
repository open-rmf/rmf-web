from typing import cast
from unittest.mock import Mock

from rmf_door_msgs.msg import DoorMode as RmfDoorMode

from api_server.test import AppFixture, make_building_map, make_door_state
from api_server.test.test_fixtures import try_until


class TestDoorsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()

    def test_get_doors(self):
        self.app.rmf_events().building_map.on_next(make_building_map())
        resp = self.session.get("/doors")
        self.assertEqual(200, resp.status_code)
        doors = resp.json()
        self.assertEqual(1, len(doors))
        self.assertEqual("test_door", doors[0]["name"])

    def test_get_door_state(self):
        self.app.rmf_events().door_states.on_next(make_door_state("test_door"))
        resp = self.session.get("/doors/test_door/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual("test_door", state["door_name"])

    def test_watch_door_state(self):
        door_state = make_door_state("test_door")
        door_state.door_time.sec = 1
        fut = self.subscribe_sio("/doors/test_door/state")

        def wait():
            self.app.rmf_events().door_states.on_next(door_state)
            return fut.done()

        try_until(wait, lambda x: x)
        result = fut.result(0)
        self.assertEqual(1, result["door_time"]["sec"])

    def test_post_door_request(self):
        resp = self.session.post(
            "/doors/test_door/request", json={"mode": RmfDoorMode.MODE_OPEN}
        )
        self.assertEqual(resp.status_code, 200)
        mock = cast(Mock, self.app.rmf_gateway().request_door)
        mock.assert_called()
        self.assertEqual("test_door", mock.call_args.args[0])
        self.assertEqual(RmfDoorMode.MODE_OPEN, mock.call_args.args[1])
