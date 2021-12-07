from uuid import uuid4

from api_server.test import AppFixture
from api_server.test.test_data import make_task_state


class TestTasksRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.tasks = [make_task_state(task_id=f"test_{uuid4()}")]

        async def prepare_db():
            for t in cls.tasks:
                await t.save()

        cls.run_in_app_loop(prepare_db())

    def test_get_task_state(self):
        resp = self.session.get(f"/tasks/{self.tasks[0].booking.id}/state")
        self.assertEqual(200, resp.status_code)
        self.assertEqual(self.tasks[0].booking.id, resp.json()["booking"]["id"])

    def test_query_task_state(self):
        resp = self.session.get(f"/tasks?task_id={self.tasks[0].booking.id}")
        self.assertEqual(200, resp.status_code)
        results = resp.json()
        self.assertEqual(1, len(results))
        self.assertEqual(self.tasks[0].booking.id, results[0]["booking"]["id"])
