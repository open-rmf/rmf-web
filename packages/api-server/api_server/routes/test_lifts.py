from unittest.mock import Mock

from rmf_lift_msgs.msg import LiftRequest as RmfLiftRequest

from api_server.test.test_data import make_building_map
from api_server.test.test_fixtures import AppFixture


class TestLiftsRoute(AppFixture):
    async def asyncSetUp(self):
        await super().asyncSetUp()
        self.app.rmf_events.building_map.on_next(make_building_map())

    async def test_get_lifts(self):
        resp = await self.client.get("/lifts")
        self.assertEqual(200, resp.status_code)
        lifts = resp.json()
        self.assertEqual(1, len(lifts))
        self.assertEqual("test_lift", lifts[0]["name"])

    async def test_lift_request(self):
        resp = await self.client.post(
            "/lifts/test_lift/request",
            json={
                "request_type": RmfLiftRequest.REQUEST_AGV_MODE,
                "door_mode": RmfLiftRequest.DOOR_OPEN,
                "destination": "L1",
            },
        )
        self.assertEqual(resp.status_code, 200)
        mock: Mock = self.rmf_gateway.request_lift
        self.assertEqual(mock.call_args.args[0], "test_lift")
        self.assertEqual(mock.call_args.args[1], "L1")
        self.assertEqual(mock.call_args.args[2], RmfLiftRequest.REQUEST_AGV_MODE)
        self.assertEqual(mock.call_args.args[3], RmfLiftRequest.DOOR_OPEN)
