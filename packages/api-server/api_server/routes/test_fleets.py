from uuid import uuid4

from api_server.rmf_io import fleet_events
from api_server.test import AppFixture, make_fleet_log, make_fleet_state


class TestFleetsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        names = [uuid4() for _ in range(2)]
        cls.fleet_states = [make_fleet_state(f"test_{x}") for x in names]
        cls.fleet_logs = [make_fleet_log(f"test_{x}") for x in names]

        async def prepare_db():
            for x in cls.fleet_states:
                await x.save()
            for x in cls.fleet_logs:
                await x.save()

        cls.run_in_app_loop(prepare_db())

    def test_query_fleets(self):
        resp = self.session.get(f"/fleets?fleet_name={self.fleet_states[0].name}")
        self.assertEqual(200, resp.status_code)
        resp_json = resp.json()
        self.assertEqual(2, len(resp_json))
        self.assertEqual(self.fleet_states[0].name, resp_json[0]["name"])

    def test_get_fleet_state(self):
        resp = self.session.get(f"/fleets/{self.fleet_states[0].name}/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual(self.fleet_states[0].name, state["name"])

    def test_sub_fleet_state(self):
        fut = self.subscribe_sio(f"/fleets/{self.fleet_states[0].name}/state")
        result = fut.result(1)
        self.assertEqual(self.fleet_states[0].name, result["name"])

    def test_get_fleet_log(self):
        # Since there are no sample fleet logs, we cannot check the log contents
        resp = self.session.get(f"/fleets/{self.fleet_logs[0].name}/log")
        self.assertEqual(200, resp.status_code)
        self.assertEqual(self.fleet_logs[0].name, resp.json()["name"])

    def test_sub_fleet_log(self):
        fleet = f"fleet_{uuid4()}"
        fut = self.subscribe_sio(f"/fleets/{fleet}/log")
        fleet_logs = make_fleet_log(fleet)
        fleet_events.fleet_logs.on_next(fleet_logs)
        result = fut.result(1)
        self.assertEqual(fleet, result["name"])
