import asyncio

from api_server.models import TaskSummary
from api_server.models import tortoise_models as ttm
from api_server.test.test_fixtures import AppFixture


class TestPaginationQuery(AppFixture):
    async def asyncSetUp(self):
        await super().asyncSetUp()
        dataset = [
            TaskSummary(
                task_id=f"task_{i}",
                task_profile={"description": {"priority": {"value": i}}},
            )
            for i in range(200)
        ]

        await asyncio.gather(
            *(ttm.TaskSummary.save_pydantic(t, "test_group") for t in dataset)
        )

    async def test_limit_results(self):
        resp = await self.client.get("/tasks")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 100)

    async def test_offset(self):
        resp = await self.client.get("/tasks?offset=150")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 50)
        self.assertEqual(resp_json[0]["summary"]["task_id"], "task_150")

    async def test_order_by(self):
        resp = await self.client.get("/tasks?order_by=-priority")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 100)
        self.assertEqual(resp_json[0]["summary"]["task_id"], "task_199")

    async def test_order_by_mapped_fields(self):
        resp = await self.client.get("/tasks?order_by=-task_id")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 100)
        self.assertEqual(resp_json[0]["summary"]["task_id"], "task_99")

    async def test_limit(self):
        resp = await self.client.get("/tasks?limit=10")
        self.assertEqual(resp.status_code, 200)
        resp_json = resp.json()
        self.assertEqual(len(resp_json), 10)

    async def test_max_limit(self):
        resp = await self.client.get("/tasks?limit=101")
        self.assertEqual(resp.status_code, 422)
