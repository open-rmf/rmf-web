from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary

from api_server.models import FleetState, RobotState, TaskSummary
from api_server.test import AppFixture, try_until
from api_server.test.test_data import make_fleet_state


class TestFleetsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        fleet_states = [
            FleetState(
                name="fleet_1",
                robots=[RobotState(name="robot_1"), RobotState(name="robot_2")],
            ),
            FleetState(
                name="fleet_2",
                robots=[RobotState(name="robot_3")],
            ),
        ]
        tasks = [
            TaskSummary(
                task_id="task_1",
                fleet_name="fleet_1",
                robot_name="robot_1",
                state=RmfTaskSummary.STATE_ACTIVE,
            ),
            TaskSummary(
                task_id="task_2",
                fleet_name="fleet_1",
                robot_name="robot_1",
                state=RmfTaskSummary.STATE_PENDING,
            ),
        ]

        for f in fleet_states:
            cls.app.rmf_events().fleet_states.on_next(f)
        for t in tasks:
            cls.app.rmf_events().task_summaries.on_next(t)

    def test_get_fleets(self):
        resp = self.session.get("/fleets?fleet_name=fleet_1")
        self.assertEqual(200, resp.status_code)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 1)

    def test_get_robots(self):
        resp = self.session.get("/fleets/robots?fleet_name=fleet_1&robot_name=robot_1")
        self.assertEqual(200, resp.status_code)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 1)
        self.assertEqual(len(resp_json[0]["tasks"]), 2)

    def test_get_fleet_state(self):
        resp = self.session.get("/fleets/fleet_1/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual("fleet_1", state["name"])

    def test_sub_fleet_state(self):
        fleet_state = make_fleet_state()
        fut = self.subscribe_sio("/fleets/test_fleet/state")

        def wait():
            self.app.rmf_events().fleet_states.on_next(fleet_state)
            return fut.done()

        try_until(wait, lambda x: x)
        result = fut.result(0)
        self.assertEqual(1, len(result["robots"]))
