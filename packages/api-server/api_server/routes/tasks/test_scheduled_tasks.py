from datetime import datetime, timedelta

from api_server.test import AppFixture


class TestScheduledTasksRoute(AppFixture):
    def test_scheduled_task_crud(self):
        task_until = (datetime.now() + timedelta(days=30)).timestamp()
        resp = self.client.get(f"/scheduled_tasks?start_from=1000&until={task_until}")
        before = resp.json()

        scheduled_task = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "period": "day",
                    "start_from": 1000,
                    "until": task_until,
                },
                {
                    "period": "day",
                    "start_from": 0,
                    "until": 999,
                },
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task)
        self.assertEqual(201, resp.status_code, resp.json())
        task = resp.json()
        self.assertEqual(len(task.schedules), 2)

        resp = self.client.get(f"/scheduled_tasks?start_from=1000&until={task_until}")
        self.assertEqual(200, resp.status_code, resp.json())
        after = resp.json()

        self.assertEqual(len(after) - len(before), 1)

        task_id = after[0]["id"]
        resp = self.client.get(f"/scheduled_tasks/{task_id}")
        self.assertEqual(200, resp.status_code)

        resp = self.client.delete(f"/scheduled_tasks/{task_id}")
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks/{task_id}")
        self.assertEqual(404, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks?start_from=1000&until={task_until}")
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
