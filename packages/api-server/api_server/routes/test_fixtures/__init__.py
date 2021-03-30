import asyncio
import unittest
from concurrent.futures import Future
from threading import Thread

import rclpy
import rclpy.node
from fastapi.testclient import TestClient

from ...app import app


class RouteFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rcl_ctx = rclpy.Context()
        rclpy.init(context=cls.rcl_ctx)

        cls.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(cls.loop)
        cls.client = TestClient(app)
        cls.client.__enter__()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown(context=cls.rcl_ctx)
        asyncio.set_event_loop(cls.loop)
        cls.client.__exit__()

    def setUp(self):
        self.node = rclpy.node.Node("test_node", context=self.rcl_ctx)

    def tearDown(self):
        self.node.destroy_node()

    def subscribe_one(self, Message, topic: str) -> Future:
        """
        returns a future that is set when the first subscription message is received
        """
        fut = Future()

        def on_msg(msg):
            self.node.destroy_subscription(sub)
            fut.set_result(msg)

        def spin():
            while not fut.done():
                rclpy.spin_once(self.node, timeout_sec=1)

        sub = self.node.create_subscription(Message, topic, on_msg, 1)
        Thread(target=spin).start()
        return fut
