# import asyncio

# from api_server.models.ros_pydantic.rmf_task_msgs import (
#     Priority,
#     TaskDescription,
#     TaskProfile,
# )
# from api_server.test import AppFixture


# class TestPaginationQuery(AppFixture):
#     @classmethod
#     def setUpClass(cls):
#         super().setUpClass()
#         dataset = [
#             TaskSummary(
#                 task_id=f"task_{i}",
#                 task_profile=TaskProfile(
#                     description=TaskDescription(priority=Priority(value=i))
#                 ),
#             )
#             for i in range(200)
#         ]

#         async def save_db():
#             await asyncio.gather(*(t.save("test_group") for t in dataset))

#         cls.run_in_app_loop(save_db())

#     def test_limit_results(self):
#         resp = self.session.get("/tasks")
#         self.assertEqual(resp.status_code, 200)
#         resp_json = resp.json()
#         self.assertEqual(len(resp_json), 100)

#     def test_offset(self):
#         resp = self.session.get("/tasks?offset=150")
#         self.assertEqual(resp.status_code, 200)
#         resp_json = resp.json()
#         self.assertEqual(len(resp_json), 50)
#         self.assertEqual(resp_json[0]["summary"]["task_id"], "task_150")

#     def test_order_by(self):
#         resp = self.session.get("/tasks?order_by=-priority")
#         self.assertEqual(resp.status_code, 200)
#         resp_json = resp.json()
#         self.assertEqual(len(resp_json), 100)
#         self.assertEqual(resp_json[0]["summary"]["task_id"], "task_199")

#     def test_order_by_mapped_fields(self):
#         resp = self.session.get("/tasks?order_by=-task_id")
#         self.assertEqual(resp.status_code, 200)
#         resp_json = resp.json()
#         self.assertEqual(len(resp_json), 100)
#         self.assertEqual(resp_json[0]["summary"]["task_id"], "task_99")

#     def test_limit(self):
#         resp = self.session.get("/tasks?limit=10")
#         self.assertEqual(resp.status_code, 200)
#         resp_json = resp.json()
#         self.assertEqual(len(resp_json), 10)

#     def test_max_limit(self):
#         resp = self.session.get("/tasks?limit=101")
#         self.assertEqual(resp.status_code, 422)
