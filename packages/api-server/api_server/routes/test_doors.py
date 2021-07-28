from unittest.mock import Mock

from rmf_door_msgs.msg import DoorMode as RmfDoorMode

from api_server.test.test_data import make_building_map, make_door_state
from api_server.test.test_fixtures import AppFixture


class TestDoorsRoute(AppFixture):
    async def asyncSetUp(self):
        await super().asyncSetUp()
        self.app.rmf_events.building_map.on_next(make_building_map())

    async def test_get_doors(self):
        resp = await self.client.get("/doors")
        self.assertEqual(200, resp.status_code)
        doors = resp.json()
        self.assertEqual(1, len(doors))
        self.assertEqual("test_door", doors[0]["name"])

    async def test_get_door_state(self):
        self.app.rmf_events.door_states.on_next(make_door_state("test_door"))
        resp = await self.client.get("/doors/test_door/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual("test_door", state["door_name"])

    async def test_post_door_request(self):
        resp = await self.client.post(
            "/doors/test_door/request", json={"mode": RmfDoorMode.MODE_OPEN}
        )
        self.assertEqual(resp.status_code, 200)
        mock: Mock = self.rmf_gateway.request_door
        mock.assert_called()
        self.assertEqual("test_door", mock.call_args.args[0])
        self.assertEqual(RmfDoorMode.MODE_OPEN, mock.call_args.args[1])
