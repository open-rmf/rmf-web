from api_server.models import Dispenser
from api_server.test.test_data import make_dispenser_state
from api_server.test.test_fixtures import AppFixture


class TestDispensersRoute(AppFixture):
    async def test_smoke(self):
        # get dispensers
        self.app.rmf_events().dispenser_states.on_next(make_dispenser_state())
        resp = await self.client.get("/dispensers")
        self.assertEqual(resp.status_code, 200)
        dispensers = [Dispenser(**d) for d in resp.json()]
        self.assertEqual(1, len(dispensers))
        self.assertEqual("test_dispenser", dispensers[0].guid)

        # get dispenser state
        self.app.rmf_events().dispenser_states.on_next(make_dispenser_state())
        resp = await self.client.get("/dispensers/test_dispenser/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual("test_dispenser", state["guid"])
