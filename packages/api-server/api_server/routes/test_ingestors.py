from api_server.models import Ingestor
from api_server.test import AppFixture, make_ingestor_state, try_until


class TestIngestorsRoute(AppFixture):
    def test_get_ingestors(self):
        self.app.rmf_events().ingestor_states.on_next(make_ingestor_state())
        resp = self.session.get("/ingestors")
        self.assertEqual(resp.status_code, 200)
        ingestors = [Ingestor(**d) for d in resp.json()]
        self.assertEqual(1, len(ingestors))
        self.assertEqual("test_ingestor", ingestors[0].guid)

    def test_get_ingestor_state(self):
        self.app.rmf_events().ingestor_states.on_next(make_ingestor_state())
        resp = self.session.get("/ingestors/test_ingestor/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual("test_ingestor", state["guid"])

    def test_watch_ingestor_state(self):
        ingestor_state = make_ingestor_state()
        ingestor_state.time.sec = 1
        fut = self.subscribe_sio("/ingestors/test_ingestor/state")

        def wait():
            self.app.rmf_events().ingestor_states.on_next(ingestor_state)
            return fut.done()

        try_until(wait, lambda x: x)
        result = fut.result(0)
        self.assertEqual(1, result["time"]["sec"])
