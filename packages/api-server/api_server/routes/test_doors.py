from unittest.mock import Mock

from rmf_door_msgs.msg import DoorMode as RmfDoorMode

from api_server.test.test_data import make_building_map, make_door_state
from api_server.test.test_fixtures import AppFixture


class TestDoorsRoute(AppFixture):
    async def test_smoke(self):
        # get doors
        self.app.rmf_events().building_map.on_next(make_building_map())
        resp = await self.client.get("/doors")
        self.assertEqual(200, resp.status_code)
        doors = resp.json()
        self.assertEqual(1, len(doors))
        self.assertEqual("test_door", doors[0]["name"])

        # get door state
        self.app.rmf_events().door_states.on_next(make_door_state("test_door"))
        resp = await self.client.get("/doors/test_door/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual("test_door", state["door_name"])

        # post door request
        resp = await self.client.post(
            "/doors/test_door/request", json={"mode": RmfDoorMode.MODE_OPEN}
        )
        self.assertEqual(resp.status_code, 200)
        mock = self.app.rmf_gateway().request_door
        mock: Mock
        mock.assert_called()
        self.assertEqual("test_door", mock.call_args.args[0])
        self.assertEqual(RmfDoorMode.MODE_OPEN, mock.call_args.args[1])
