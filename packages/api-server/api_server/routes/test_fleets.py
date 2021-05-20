from rmf_fleet_msgs.msg import FleetState as RmfFleetState

from .test_fixtures import RouteFixture


class TestFleetsRoute(RouteFixture):
    def test_get_fleets(self):
        resp = self.session.get(f"{self.base_url}/fleets")
        self.assertEqual(resp.status_code, 200)

    def test_get_fleet_state(self):
        pub = self.node.create_publisher(RmfFleetState, "fleet_states", 10)
        rmf_ingestor_state = RmfFleetState(name="test_fleet")
        pub.publish(rmf_ingestor_state)
        resp = self.session.get(f"{self.base_url}/fleets/test_fleet/state")
        self.assertEqual(resp.status_code, 200)
