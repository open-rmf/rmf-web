from uuid import uuid4

from api_server.test import AppFixture, make_task_state, try_until


class TestTasksRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.task_states = [make_task_state(task_id=f"test_{uuid4()}")]

        async def prepare_db():
            for t in cls.task_states:
                await t.save()

        cls.run_in_app_loop(prepare_db())

    def test_get_task_state(self):
        resp = self.session.get(f"/tasks/{self.task_states[0].booking.id}/state")
        self.assertEqual(200, resp.status_code)
        self.assertEqual(self.task_states[0].booking.id, resp.json()["booking"]["id"])

    def test_query_task_states(self):
        resp = self.session.get(f"/tasks?task_id={self.task_states[0].booking.id}")
        self.assertEqual(200, resp.status_code)
        results = resp.json()
        self.assertEqual(1, len(results))
        self.assertEqual(self.task_states[0].booking.id, results[0]["booking"]["id"])

    def test_sub_task_state(self):
        fut = self.subscribe_sio(f"/tasks/{self.task_states[0].booking.id}/state")

        def wait():
            return fut.done()

        try_until(wait, lambda x: x)
        result = fut.result(0)
        self.assertEqual(self.task_states[0].booking.id, result["booking"]["id"])
