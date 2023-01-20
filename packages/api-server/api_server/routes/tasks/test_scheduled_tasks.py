from api_server.test import AppFixture


class TestScheduledTasksRoute(AppFixture):
    def test_post_scheduled_task(self):
        resp = self.client.get("/scheduled_tasks")
        before = len(resp.json())

        scheduled_task = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "period": "day",
                }
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task)
        self.assertEqual(201, resp.status_code)

        resp = self.client.get("/scheduled_tasks")
        after = len(resp.json())

        self.assertEqual(after - before, 1)
