from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary

from api_server.models import TaskSummary
from api_server.models.fleets import FleetState, RobotState
from api_server.test.test_fixtures import AppFixture


class TestFleetsRoute(AppFixture):
    async def asyncSetUp(self):
        await super().asyncSetUp()
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
            self.app.rmf_events().fleet_states.on_next(f)
        for t in tasks:
            self.app.rmf_events().task_summaries.on_next(t)

    async def test_smoke(self):
        # get fleets
        resp = await self.client.get("/fleets?fleet_name=fleet_1")
        self.assertEqual(200, resp.status_code)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 1)

        # get robots
        resp = await self.client.get(
            "/fleets/robots?fleet_name=fleet_1&robot_name=robot_1"
        )
        self.assertEqual(200, resp.status_code)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 1)
        self.assertEqual(len(resp_json[0]["tasks"]), 2)

        # get fleet state
        resp = await self.client.get("/fleets/fleet_1/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual("fleet_1", state["name"])
