from uuid import uuid4

from api_server.models import FleetLogUpdate, FleetStateUpdate
from api_server.test import AppFixture, make_fleet_log, make_fleet_state


class TestFleetsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        names = [uuid4() for _ in range(2)]
        cls.fleet_states = [make_fleet_state(f"test_{x}") for x in names]
        cls.fleet_logs = [make_fleet_log(f"test_{x}") for x in names]

        with cls.client.websocket_connect("/_internal") as ws:
            for x in cls.fleet_states:
                ws.send_text(FleetStateUpdate(type="fleet_state_update", data=x).json())
            for x in cls.fleet_logs:
                ws.send_text(FleetLogUpdate(type="fleet_log_update", data=x).json())

    def test_query_fleets(self):
        resp = self.client.get(f"/fleets?fleet_name={self.fleet_states[0].name}")
        self.assertEqual(200, resp.status_code)
        resp_json = resp.json()
        self.assertEqual(2, len(resp_json))
        self.assertEqual(self.fleet_states[0].name, resp_json[0]["name"])

    def test_get_fleet_state(self):
        resp = self.client.get(f"/fleets/{self.fleet_states[0].name}/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual(self.fleet_states[0].name, state["name"])

    def test_sub_fleet_state(self):
        fleet = self.fleet_states[0].name
        gen = self.subscribe_sio(f"/fleets/{fleet}/state")

        with self.client.websocket_connect("/_internal") as ws:
            ws.send_text(
                FleetStateUpdate(
                    type="fleet_state_update", data=self.fleet_states[0]
                ).json()
            )

        msg = next(gen)
        self.assertEqual(fleet, msg.name)

    def test_get_fleet_log(self):
        # Since there are no sample fleet logs, we cannot check the log contents
        resp = self.client.get(f"/fleets/{self.fleet_logs[0].name}/log")
        self.assertEqual(200, resp.status_code)
        self.assertEqual(self.fleet_logs[0].name, resp.json()["name"])

    def test_sub_fleet_log(self):
        fleet = self.fleet_logs[0].name
        gen = self.subscribe_sio(f"/fleets/{fleet}/log")

        with self.client.websocket_connect("/_internal") as ws:
            ws.send_text(
                FleetLogUpdate(type="fleet_log_update", data=self.fleet_logs[0]).json()
            )

        msg = next(gen)
        self.assertEqual(fleet, msg.name)
