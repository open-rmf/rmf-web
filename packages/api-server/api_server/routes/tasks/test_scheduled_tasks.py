from datetime import datetime, timedelta

from api_server.test import AppFixture


class TestScheduledTasksRoute(AppFixture):
    def test_scheduled_task_crud(self):
        task_until = (datetime.now() + timedelta(days=30)).timestamp()

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
                {
                    "period": "monday",
                    "start_from": 1000,
                    "until": task_until,
                },
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task)
        self.assertEqual(201, resp.status_code, resp.json())
        task1 = resp.json()
        self.assertEqual(len(task1["schedules"]), 3, task1)

        scheduled_task_2 = {
            "task_request": {
                "category": "test",
                "description": "test",
            },
            "schedules": [
                {
                    "period": "day",
                    "start_from": 2000,
                    "until": task_until,
                },
            ],
        }
        resp = self.client.post("/scheduled_tasks", json=scheduled_task_2)
        self.assertEqual(201, resp.status_code, resp.json())
        task2 = resp.json()
        self.assertEqual(len(task2["schedules"]), 1, task2)

        # check each task id only appears once
        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until}"
        )
        self.assertEqual(200, resp.status_code)
        task_ids = [x["id"] for x in resp.json()]
        unique_ids = set(task_ids)
        self.assertEqual(
            len(task_ids), len(unique_ids), "one or more task appears multiple times"
        )

        resp = self.client.get(
            f"/scheduled_tasks?start_before=1000&until_after={task_until}"
        )
        self.assertEqual(200, resp.status_code, resp.json())
        tasks = {x["id"]: x for x in resp.json()}
        self.assertIn(task1["id"], tasks)
        # task2 starts after `start_before`` so should not be included
        self.assertNotIn(task2["id"], tasks)

        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until}"
        )
        self.assertEqual(200, resp.status_code, resp.json())
        after = resp.json()
        tasks = {x["id"]: x for x in after}
        self.assertIn(task1["id"], tasks)
        self.assertIn(task2["id"], tasks)

        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until+1}"
        )
        self.assertEqual(200, resp.status_code, resp.json())
        after = resp.json()
        tasks = {x["id"]: x for x in after}
        # neither task should be returned as they stop before `until_after`
        self.assertNotIn(task1["id"], tasks)
        self.assertNotIn(task2["id"], tasks)

        resp = self.client.get(f"/scheduled_tasks/{task1['id']}")
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks/{task2['id']}")
        self.assertEqual(200, resp.status_code)

        resp = self.client.delete(f"/scheduled_tasks/{task1['id']}")
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks/{task1['id']}")
        self.assertEqual(404, resp.status_code)
        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until}"
        )
        tasks = {x["id"]: x for x in resp.json()}
        self.assertNotIn(task1["id"], tasks)
        # task 2 should not be deleted
        self.assertIn(task2["id"], tasks)

        resp = self.client.delete(f"/scheduled_tasks/{task2['id']}")
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/scheduled_tasks/{task2['id']}")
        self.assertEqual(404, resp.status_code)
        resp = self.client.get(
            f"/scheduled_tasks?start_before=2000&until_after={task_until}"
        )
        tasks = {x["id"]: x for x in resp.json()}
        self.assertNotIn(task1["id"], tasks)
        self.assertNotIn(task2["id"], tasks)

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

    def test_get_scheduled_tasks_return_indefinite_tasks(self):
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
        self.assertEqual(201, resp.status_code, resp.json())
        task = resp.json()

        resp = self.client.get("/scheduled_tasks?start_before=0&until_after=0")
        self.assertEqual(200, resp.status_code, resp.json())
        tasks = {x["id"]: x for x in resp.json()}
        self.assertIn(task["id"], tasks)
