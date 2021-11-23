from api_server.models import Dispenser
from api_server.test import AppFixture, make_dispenser_state, try_until


class TestDispensersRoute(AppFixture):
    def test_get_dispensers(self):
        self.app.rmf_events().dispenser_states.on_next(make_dispenser_state())
        resp = self.session.get("/dispensers")
        self.assertEqual(resp.status_code, 200)
        dispensers = [Dispenser(**d) for d in resp.json()]
        self.assertEqual(1, len(dispensers))
        self.assertEqual("test_dispenser", dispensers[0].guid)

    def test_get_dispenser_state(self):
        self.app.rmf_events().dispenser_states.on_next(make_dispenser_state())
        resp = self.session.get("/dispensers/test_dispenser/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual("test_dispenser", state["guid"])

    def test_watch_dispenser_state(self):
        dispenser_state = make_dispenser_state()
        dispenser_state.time.sec = 1
        fut = self.subscribe_sio("/dispensers/test_dispenser/state")

        def wait():
            self.app.rmf_events().dispenser_states.on_next(dispenser_state)
            return fut.done()

        try_until(wait, lambda x: x)
        result = fut.result(0)
        self.assertEqual(1, result["time"]["sec"])
