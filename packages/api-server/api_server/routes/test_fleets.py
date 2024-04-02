from api_server.models import FleetLog, FleetLogUpdate, FleetState, FleetStateUpdate
from api_server.rmf_io.events import fleet_events
from api_server.test import AppFixture, make_fleet_log, make_fleet_state


class TestFleetsRoute(AppFixture):
    def test_fleet_states(self):
        fleet_state = make_fleet_state()

        with self.client.websocket_connect("/_internal") as ws, self.subscribe_sio(
            f"/fleets/{fleet_state.name}/state"
        ) as sub:
            ws.send_text(
                FleetStateUpdate(
                    type="fleet_state_update", data=fleet_state
                ).model_dump_json()
            )

            msg = FleetState(**next(sub))
            self.assertEqual(fleet_state.name, msg.name)

            # get fleet state
            resp = self.client.get(f"/fleets/{fleet_state.name}/state")
            self.assertEqual(200, resp.status_code)
            state = resp.json()
            self.assertEqual(fleet_state.name, state["name"])

            # query fleets
            resp = self.client.get(f"/fleets?fleet_name={fleet_state.name}")
            self.assertEqual(200, resp.status_code)
            resp_json = resp.json()
            self.assertEqual(1, len(resp_json))
            self.assertEqual(fleet_state.name, resp_json[0]["name"])

    def test_fleet_logs(self):
        fleet_log = make_fleet_log()

        with self.client.websocket_connect("/_internal") as ws, self.subscribe_sio(
            f"/fleets/{fleet_log.name}/log"
        ) as sub:
            fleet_events.fleet_logs.on_next(fleet_log)

            ws.send_text(
                FleetLogUpdate(
                    type="fleet_log_update", data=fleet_log
                ).model_dump_json()
            )

            msg = FleetLog(**next(sub))
            self.assertEqual(fleet_log.name, msg.name)

            # Since there are no sample fleet logs, we cannot check the log contents
            resp = self.client.get(f"/fleets/{fleet_log.name}/log")
            self.assertEqual(200, resp.status_code)
            self.assertEqual(fleet_log.name, resp.json()["name"])
