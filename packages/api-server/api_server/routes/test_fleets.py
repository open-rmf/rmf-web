from uuid import uuid4

from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary

from api_server.models import FleetState, RobotState, TaskSummary
from api_server.rmf_io import rmf_events
from api_server.test import AppFixture, try_until


class TestFleetsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.fleets = [f"fleet_{uuid4()}", f"fleet_{uuid4()}"]
        fleet_states = [
            FleetState(
                name=cls.fleets[0],
                robots=[RobotState(name="robot_1"), RobotState(name="robot_2")],
            ),
            FleetState(
                name=cls.fleets[1],
                robots=[RobotState(name="robot_3")],
            ),
        ]
        tasks = [
            TaskSummary(
                task_id=f"task_{uuid4()}",
                fleet_name=cls.fleets[0],
                robot_name="robot_1",
                state=RmfTaskSummary.STATE_ACTIVE,
            ),
            TaskSummary(
                task_id=f"task_{uuid4()}",
                fleet_name=cls.fleets[0],
                robot_name="robot_1",
                state=RmfTaskSummary.STATE_PENDING,
            ),
        ]

        for f in fleet_states:
            rmf_events.fleet_states.on_next(f)
        for t in tasks:
            rmf_events.task_summaries.on_next(t)

    def test_get_fleets(self):
        resp = try_until(
            lambda: self.session.get(f"/fleets?fleet_name={self.fleets[0]}"),
            lambda x: x.status_code == 200 and len(x.json()) == 1,
        )
        self.assertEqual(200, resp.status_code)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 1)

    def test_get_robots(self):
        resp = try_until(
            lambda: self.session.get(
                f"/fleets/robots?fleet_name={self.fleets[0]}&robot_name=robot_1"
            ),
            lambda x: x.status_code == 200 and len(x.json()) == 1,
        )
        self.assertEqual(200, resp.status_code)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 1)
        self.assertEqual(len(resp_json[0]["tasks"]), 2)

    def test_get_fleet_state(self):
        resp = try_until(
            lambda: self.session.get(f"/fleets/{self.fleets[0]}/state"),
            lambda x: x.status_code == 200,
        )
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual(self.fleets[0], state["name"])

    def test_sub_fleet_state(self):
        fut = self.subscribe_sio(f"/fleets/{self.fleets[0]}/state")
        try_until(fut.done, lambda x: x)
        result = fut.result(0)
        self.assertEqual(self.fleets[0], result["name"])
