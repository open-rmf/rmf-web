from urllib.parse import urlencode

from api_server.models import (
    FleetLog,
    FleetLogUpdate,
    FleetState,
    FleetStateUpdate,
    MutexGroups,
)
from api_server.rmf_io import fleet_events
from api_server.test import (
    AppFixture,
    make_fleet_log,
    make_fleet_state,
    make_robot_state,
)


class TestFleetsRoute(AppFixture):
    def test_fleet_states(self):
        fleet_state = make_fleet_state("test_fleet")

        with self.client.websocket_connect("/_internal") as ws, self.subscribe_sio(
            f"/fleets/{fleet_state.name}/state"
        ) as sub:
            ws.send_text(
                FleetStateUpdate(type="fleet_state_update", data=fleet_state).json()
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
            self.assertEqual(1, len(resp_json), resp_json)
            self.assertEqual(fleet_state.name, resp_json[0]["name"])

    def test_fleet_logs(self):
        fleet_log = make_fleet_log()

        with self.client.websocket_connect("/_internal") as ws, self.subscribe_sio(
            f"/fleets/{fleet_log.name}/log"
        ) as sub:
            fleet_events.fleet_logs.on_next(fleet_log)

            ws.send_text(FleetLogUpdate(type="fleet_log_update", data=fleet_log).json())

            msg = FleetLog(**next(sub))
            self.assertEqual(fleet_log.name, msg.name)

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

    def test_unlock_mutex_group(self):
        # add a new robot
        robot_name = "test_robot"
        robot_state = make_robot_state(robot_name)
        fleet_state = make_fleet_state("test_fleet")
        robot_state.mutex_groups = MutexGroups(
            locked=["test_locked_mutex_group"], requesting=[]
        )
        fleet_state.robots = {robot_name: robot_state}

        with self.client.websocket_connect("/_internal") as ws:
            ws.send_text(
                FleetStateUpdate(type="fleet_state_update", data=fleet_state).json()
            )

        # valid
        params = {"robot_name": "test_robot", "mutex_group": "test_locked_mutex_group"}
        resp = self.client.post(
            f"fleets/{fleet_state.name}/unlock_mutex_group?{urlencode(params)}"
        )
        self.assertEqual(200, resp.status_code)

        # invalid fleet
        params = {"robot_name": robot_name, "mutex_group": "test_locked_mutex_group"}
        resp = self.client.post(
            f"fleets/invalid_fleet/unlock_mutex_group?{urlencode(params)}"
        )
        self.assertEqual(404, resp.status_code)

        # invalid robot
        params = {"robot_name": "", "mutex_group": "test_locked_mutex_group"}
        resp = self.client.post(
            f"fleets/{fleet_state.name}/unlock_mutex_group?{urlencode(params)}"
        )
        self.assertEqual(404, resp.status_code)
        params = {
            "robot_name": "invalid_robot",
            "mutex_group": "test_locked_mutex_group",
        }
        resp = self.client.post(
            f"fleets/{fleet_state.name}/unlock_mutex_group?{urlencode(params)}"
        )
        self.assertEqual(404, resp.status_code)

        # invalid mutex group
        params = {"robot_name": "test_robot", "mutex_group": ""}
        resp = self.client.post(
            f"fleets/{fleet_state.name}/unlock_mutex_group?{urlencode(params)}"
        )
        self.assertEqual(400, resp.status_code)
        params = {"robot_name": "test_robot", "mutex_group": "invalid_mutex_group"}
        resp = self.client.post(
            f"fleets/{fleet_state.name}/unlock_mutex_group?{urlencode(params)}"
        )
        self.assertEqual(400, resp.status_code)
