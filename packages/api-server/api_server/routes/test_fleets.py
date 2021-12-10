from uuid import uuid4

from api_server.test import AppFixture, make_fleet_state, try_until


class TestFleetsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.fleet_states = [
            make_fleet_state(f"test_{uuid4()}"),
            make_fleet_state(f"test_{uuid4()}"),
        ]

        async def prepare_db():
            for x in cls.fleet_states:
                await x.save()

        cls.run_in_app_loop(prepare_db())

    def test_query_fleets(self):
        resp = self.session.get(f"/fleets?fleet_name={self.fleet_states[0].name}")
        self.assertEqual(200, resp.status_code)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 1)
        self.assertEqual(self.fleet_states[0].name, resp_json[0]["name"])

    def test_get_fleet_state(self):
        resp = self.session.get(f"/fleets/{self.fleet_states[0].name}/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual(self.fleet_states[0].name, state["name"])

    def test_sub_fleet_state(self):
        fut = self.subscribe_sio(f"/fleets/{self.fleet_states[0].name}/state")

        def wait():
            return fut.done()

        try_until(wait, lambda x: x)
        result = fut.result(0)
        self.assertEqual(self.fleet_states[0].name, result["name"])
