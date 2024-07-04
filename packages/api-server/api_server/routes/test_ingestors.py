from typing import List
from uuid import uuid4

from api_server.models import IngestorState
from api_server.models.user import User
from api_server.repositories.rmf import RmfRepository
from api_server.test import AppFixture, make_ingestor_state


class TestIngestorsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rmf_repo = RmfRepository(User.get_system_user())
        cls.ingestor_states = [make_ingestor_state(f"test_{uuid4()}")]

        portal = cls.get_portal()
        for x in cls.ingestor_states:
            portal.call(rmf_repo.save_ingestor_state, x)

    def test_get_ingestors(self):
        resp = self.client.get("/ingestors")
        self.assertEqual(resp.status_code, 200)
        results: List = resp.json()
        self.assertIsNotNone(
            next(
                (x for x in results if x["guid"] == self.ingestor_states[0].guid), None
            )
        )

    def test_get_ingestor_state(self):
        resp = self.client.get(f"/ingestors/{self.ingestor_states[0].guid}/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual(self.ingestor_states[0].guid, state["guid"])

    def test_sub_ingestor_state(self):
        with self.subscribe_sio(
            f"/ingestors/{self.ingestor_states[0].guid}/state"
        ) as sub:
            msg = IngestorState(**next(sub))
            self.assertEqual(self.ingestor_states[0].guid, msg.guid)
