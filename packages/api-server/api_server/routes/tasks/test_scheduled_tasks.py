from api_server.test import AppFixture


class TestScheduledTasksRoute(AppFixture):
    def test_scheduled_task_crud(self):
        resp = self.client.get("/scheduled_tasks")
        before = resp.json()

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
        after = resp.json()

        self.assertEqual(len(after) - len(before), 1)

        task_id = after[0]["id"]
        resp = self.client.get(f"/scheduled_tasks/{task_id}")
        self.assertEqual(200, resp.status_code)

        resp = self.client.delete(f"/scheduled_tasks/{task_id}")
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks/{task_id}")
        self.assertEqual(404, resp.status_code)
        resp = self.client.get("/scheduled_tasks")
        self.assertEqual(len(before), len(resp.json()))

    def test_cannot_create_task_that_never_runs(self):
        scheduled_task = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "start_from": 0,
                    "until": 0,
                    "period": "day",
                }
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task)
        self.assertEqual(422, resp.status_code)
