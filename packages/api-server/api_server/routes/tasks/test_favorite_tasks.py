from datetime import datetime
from uuid import uuid4

from api_server import models as mdl
from api_server.test import AppFixture, make_task_favorite


class TestFavoriteTasksRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        favorite_tasks_ids = [uuid4()]
        cls.favorites_tasks = [
            make_task_favorite(favorite_task_id=f"test_{x}") for x in favorite_tasks_ids
        ]

    def test_get_task_favorite(self):
        self.create_favorite_task()
        resp = self.client.get("/tasks/favorites_tasks")
        self.assertEqual(200, resp.status_code)
        start_time = (
            datetime.fromtimestamp(
                self.favorites_tasks[0].unix_millis_earliest_start_time / 1000
            )
            if self.favorites_tasks[0].unix_millis_earliest_start_time
            else "1636388410000"
        )
        self.assertEqual(
            int(start_time.strftime("%Y%m%d%H%M%S")),
            resp.json()[0]["unix_millis_earliest_start_time"],
        )
        return resp

    def test_favorite_create_success(self):
        resp = self.post_favorite_task()
        self.assertEqual(200, resp.status_code)
        self.assertEqual(resp.json(), None)

    def test_delete_favorite(self):
        resp_get = self.test_get_task_favorite()
        favorite_task_id = resp_get.json()[0]["id"]
        resp = self.client.delete(f"/tasks/favorite_task/{favorite_task_id}")
        self.assertEqual(200, resp.status_code)
