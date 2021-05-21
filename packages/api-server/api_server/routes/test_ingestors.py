from rmf_ingestor_msgs.msg import IngestorState as RmfIngestorState

from .test_fixtures import RouteFixture, try_until


class TestIngestorsRoute(RouteFixture):
    def test_get_ingestors(self):
        resp = self.session.get(f"{self.base_url}/ingestors")
        self.assertEqual(resp.status_code, 200)

    def test_get_ingestor_state(self):
        pub = self.node.create_publisher(RmfIngestorState, "ingestor_states", 10)
        rmf_ingestor_state = RmfIngestorState(guid="test_ingestor")

        def try_get():
            pub.publish(rmf_ingestor_state)
            return self.session.get(f"{self.base_url}/ingestors/test_ingestor/state")

        resp = try_until(
            try_get,
            lambda x: x.status_code == 200,
        )
        self.assertEqual(resp.status_code, 200)
