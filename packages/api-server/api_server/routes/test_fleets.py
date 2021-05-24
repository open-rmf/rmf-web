import asyncio
import concurrent.futures

from rmf_fleet_msgs.msg import FleetState as RmfFleetState

from api_server.models.fleets import FleetState, RobotState
from api_server.models.tasks import TaskStateEnum, TaskSummary

from ..models import tortoise_models as ttm
from .test_fixtures import RouteFixture, try_until


class TestFleetsRoute(RouteFixture):
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
                state=TaskStateEnum.ACTIVE.value,
            ),
            TaskSummary(
                task_id="task_2",
                fleet_name="fleet_1",
                robot_name="robot_1",
                state=TaskStateEnum.PENDING.value,
            ),
        ]

        fut = concurrent.futures.Future()

        async def save_data():
            await asyncio.gather(
                *[ttm.FleetState.save_pydantic(s) for s in fleet_states]
            )
            await asyncio.gather(
                *[
                    ttm.RobotState.save_pydantic(f.name, r)
                    for f in fleet_states
                    for r in f.robots
                ]
            )
            await asyncio.gather(*[ttm.TaskSummary.save_pydantic(t) for t in tasks])
            fut.set_result(True)

        cls.server.app.wait_ready()
        cls.server.app.loop.create_task(save_data())
        fut.result()

    def test_get_fleets(self):
        resp = self.session.get(f"{self.base_url}/fleets?fleet_name=fleet_1")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        self.assertEqual(resp_json["total_count"], 1)
        self.assertEqual(len(resp_json["items"]), 1)

    def test_get_robots(self):
        resp = self.session.get(
            f"{self.base_url}/fleets/robots?fleet_name=fleet_1&robot_name=robot_1"
        )
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        self.assertEqual(resp_json["total_count"], 1)
        self.assertEqual(len(resp_json["items"]), 1)
        self.assertEqual(len(resp_json["items"][0]["tasks"]), 2)


class TestFleetsRoute_RMF(RouteFixture):
    def test_get_fleet_state(self):
        pub = self.node.create_publisher(RmfFleetState, "fleet_states", 10)
        rmf_ingestor_state = RmfFleetState(name="test_fleet")

        def try_get():
            pub.publish(rmf_ingestor_state)
            return self.session.get(f"{self.base_url}/fleets/test_fleet/state")

        resp = try_until(
            try_get,
            lambda x: x.status_code == 200,
        )
        self.assertEqual(resp.status_code, 200)
