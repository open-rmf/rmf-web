from uuid import uuid4

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
        resp = self.client.get("/favorite_tasks")
        self.assertEqual(200, resp.status_code)
        return resp

    def test_favorite_create_success(self):
        resp = self.post_favorite_task()
        self.assertEqual(200, resp.status_code)
        self.assertEqual(resp.json(), None)

    def test_delete_favorite(self):
        self.create_favorite_task()
        before_delete = self.test_get_task_favorite()
        favorite_task_id = before_delete.json()[0]["id"]
        resp = self.client.delete(f"/favorite_tasks/{favorite_task_id}")
        after_delete = self.test_get_task_favorite()
        self.assertEqual(200, resp.status_code)
        self.assertEqual(
            len(before_delete.json()) - len(after_delete.json()),
            1,
        )
