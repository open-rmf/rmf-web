from rmf_ingestor_msgs.msg import IngestorState as RmfIngestorState

from .test_fixtures import RouteFixture


class TestIngestorsRoute(RouteFixture):
    def test_get_ingestors(self):
        resp = self.session.get(f"{self.base_url}/ingestors")
        self.assertEqual(resp.status_code, 200)

    def test_get_ingestor_state(self):
        pub = self.node.create_publisher(RmfIngestorState, "ingestor_states", 10)
        rmf_ingestor_state = RmfIngestorState(guid="test_ingestor")
        pub.publish(rmf_ingestor_state)
        resp = self.try_get(f"{self.base_url}/ingestors/test_ingestor/state")
        self.assertEqual(resp.status_code, 200)
