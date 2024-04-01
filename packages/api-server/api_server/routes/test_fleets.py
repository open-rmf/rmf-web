from urllib.parse import urlencode

from api_server.models import FleetLogUpdate, FleetStateUpdate
from api_server.test import (
    AppFixture,
    make_fleet_log,
    make_fleet_state,
    make_robot_state,
)


class TestFleetsRoute(AppFixture):
    def test_fleet_states(self):
        # subscribe to fleet states
        fleet_state = make_fleet_state("test_fleet")
        gen = self.subscribe_sio(f"/fleets/{fleet_state.name}/state")

        with self.client.websocket_connect("/_internal") as ws:
            ws.send_text(
                FleetStateUpdate(type="fleet_state_update", data=fleet_state).json()
            )

        msg = next(gen)
        self.assertEqual(fleet_state.name, msg.name)  # type: ignore

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
        gen = self.subscribe_sio(f"/fleets/{fleet_log.name}/log")

        with self.client.websocket_connect("/_internal") as ws:
            ws.send_text(FleetLogUpdate(type="fleet_log_update", data=fleet_log).json())

        msg = next(gen)
        self.assertEqual(fleet_log.name, msg.name)  # type: ignore

        # Since there are no sample fleet logs, we cannot check the log contents
        resp = self.client.get(f"/fleets/{fleet_log.name}/log")
        self.assertEqual(200, resp.status_code)
        self.assertEqual(fleet_log.name, resp.json()["name"])

    def test_decommission_robot(self):
        # add a new robot
        robot_name = "test_robot"
        robot_state = make_robot_state(robot_name)
        fleet_state = make_fleet_state("test_fleet")
        fleet_state.robots = {robot_name: robot_state}

        with self.client.websocket_connect("/_internal") as ws:
            ws.send_text(
                FleetStateUpdate(type="fleet_state_update", data=fleet_state).json()
            )

        # invalid fleets
        params = {
            "robot_name": robot_name,
            "reassign_tasks": True,
            "allow_idle_behavior": False,
        }
        resp = self.client.post(
            f"fleets/invalid_fleet/decommission?{urlencode(params)}"
        )
        self.assertEqual(404, resp.status_code)

        # invalid robot
        params = {
            "robot_name": "",
            "reassign_tasks": True,
            "allow_idle_behavior": False,
        }
        resp = self.client.post(
            f"fleets/{fleet_state.name}/decommission?{urlencode(params)}"
        )
        self.assertEqual(404, resp.status_code)
        params = {
            "robot_name": "invalid_robot",
            "reassign_tasks": True,
            "allow_idle_behavior": False,
        }
        resp = self.client.post(
            f"fleets/{fleet_state.name}/decommission?{urlencode(params)}"
        )
        self.assertEqual(404, resp.status_code)

    def test_recommission_robot(self):
        # add a new robot
        robot_name = "test_robot"
        robot_state = make_robot_state(robot_name)
        fleet_state = make_fleet_state("test_fleet")
        fleet_state.robots = {robot_name: robot_state}

        with self.client.websocket_connect("/_internal") as ws:
            ws.send_text(
                FleetStateUpdate(type="fleet_state_update", data=fleet_state).json()
            )

        # invalid fleet
        params = {
            "robot_name": robot_name,
        }
        resp = self.client.post(
            f"fleets/invalid_fleet/recommission?{urlencode(params)}"
        )
        self.assertEqual(404, resp.status_code)

        # invalid robot
        params = {
            "robot_name": "",
        }
        resp = self.client.post(
            f"fleets/{fleet_state.name}/recommission?{urlencode(params)}"
        )
        self.assertEqual(404, resp.status_code)
        params = {
            "robot_name": "invalid_robot",
        }
        resp = self.client.post(
            f"fleets/{fleet_state.name}/recommission?{urlencode(params)}"
        )
        self.assertEqual(404, resp.status_code)
