import asyncio
import concurrent.futures

from rmf_fleet_msgs.msg import FleetState as RmfFleetState

from api_server.models.fleets import FleetState, RobotState

from ..models import tortoise_models as ttm
from .test_fixtures import RouteFixture, try_until


class TestFleetsRoute(RouteFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        dataset = [
            FleetState(
                name="fleet_1",
                robots=[RobotState(name="robot_1"), RobotState(name="robot_2")],
            ),
            FleetState(
                name="fleet_2",
                robots=[RobotState(name="robot_3")],
            ),
        ]

        fut = concurrent.futures.Future()

        async def save_data():
            fut.set_result(
                await asyncio.gather(
                    *(ttm.FleetState.save_pydantic(data) for data in dataset)
                )
            )

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
        resp = self.session.get(f"{self.base_url}/fleets/robots")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 3)


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
