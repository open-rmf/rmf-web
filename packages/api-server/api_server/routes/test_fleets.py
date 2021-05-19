from rmf_fleet_msgs.msg import FleetState as RmfFleetState

from .test_fixtures import RouteFixture, try_until


class TestFleetsRoute(RouteFixture):
    def test_get_fleets(self):
        resp = self.session.get(f"{self.base_url}/fleets")
        self.assertEqual(resp.status_code, 200)

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
