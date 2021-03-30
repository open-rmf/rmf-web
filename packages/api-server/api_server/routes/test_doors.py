# import asyncio
# import unittest
# from concurrent.futures import Future
# from threading import Thread

# import rclpy
# import rclpy.node
# from fastapi.testclient import TestClient
# from rmf_door_msgs.msg import DoorMode, DoorRequest

# from ..app import app, logger, on_shutdown, on_startup


# class TestDoorsRoute(unittest.TestCase):
#     @classmethod
#     def setUpClass(cls):
#         cls.rcl_ctx = rclpy.Context()
#         rclpy.init(context=cls.rcl_ctx)

#         logger.setLevel("CRITICAL")
#         cls.client = TestClient(app)
#         loop = asyncio.get_event_loop()
#         loop.run_until_complete(on_startup())

#     @classmethod
#     def tearDownClass(cls):
#         loop = asyncio.get_event_loop()
#         loop.run_until_complete(on_shutdown())

#     def setUp(self):
#         self.node = rclpy.node.Node("test_node", context=self.rcl_ctx)
#         self.executor = rclpy.executors.SingleThreadedExecutor(context=self.rcl_ctx)
#         self.finish_fut = rclpy.Future(executor=self.executor)
#         self.spin_thread = Thread(
#             target=rclpy.spin_until_future_complete,
#             args=(self.node, self.finish_fut, self.executor),
#         )
#         self.spin_thread.start()

#     def tearDown(self):
#         # need a done callback for spin to stop for some reason
#         self.finish_fut.add_done_callback(lambda _: 0)
#         self.finish_fut.set_result(True)
#         self.spin_thread.join()
#         self.node.destroy_node()

#     def subscribe_one(self, Message, topic: str):
#         fut = Future()

#         def on_msg(msg):
#             self.node.destroy_subscription(sub)
#             fut.set_result(msg)

#         sub = self.node.create_subscription(Message, topic, on_msg, 1)
#         return fut

#     def test_door_request(self):
#         fut = self.subscribe_one(DoorRequest, "adapter_door_requests")
#         resp = self.client.post(
#             "/doors/test_door/request", json={"mode": DoorMode.MODE_OPEN}
#         )
#         self.assertEqual(resp.status_code, 200)
#         fut.result(1)
