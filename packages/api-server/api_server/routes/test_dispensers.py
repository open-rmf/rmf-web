from api_server.models import Dispenser
from api_server.test.test_data import make_dispenser_state
from api_server.test.test_fixtures import AppFixture


class TestDispensersRoute(AppFixture):
    async def test_get_dispensers(self):
        self.app.rmf_events.dispenser_states.on_next(make_dispenser_state())

        # should be able to see it in /dispensers
        resp = await self.client.get("/dispensers")
        self.assertEqual(resp.status_code, 200)
        dispensers = [Dispenser(**d) for d in resp.json()]
        self.assertEqual(1, len(dispensers))
        self.assertEqual("test_dispenser", dispensers[0].guid)
