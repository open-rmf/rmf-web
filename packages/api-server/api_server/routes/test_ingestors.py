from api_server.models import Ingestor
from api_server.test.test_data import make_ingestor_state
from api_server.test.test_fixtures import AppFixture


class TestIngestorsRoute(AppFixture):
    async def test_get_ingestors(self):
        self.app.rmf_events.ingestor_states.on_next(make_ingestor_state())

        # should be able to see it in /ingestors
        resp = await self.client.get("/ingestors")
        self.assertEqual(resp.status_code, 200)
        ingestors = [Ingestor(**d) for d in resp.json()]
        self.assertEqual(1, len(ingestors))
        self.assertEqual("test_ingestor", ingestors[0].guid)
