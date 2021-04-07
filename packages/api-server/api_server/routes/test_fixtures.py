import asyncio
import unittest
from concurrent.futures import Future
from threading import Thread

import rclpy
import rclpy.node
from fastapi.testclient import TestClient

from ..app import app


class RouteFixture(unittest.IsolatedAsyncioTestCase):
    @classmethod
    def setUpClass(cls):
        cls.asd = []
        cls.rcl_ctx = rclpy.Context()
        rclpy.init(context=cls.rcl_ctx)
        cls.rcl_executor = rclpy.executors.SingleThreadedExecutor(context=cls.rcl_ctx)

        cls.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(cls.loop)
        cls.client = TestClient(app)
        cls.client.__enter__()

        cls.node = rclpy.node.Node("test_node", context=cls.rcl_ctx)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.rcl_ctx)
        asyncio.set_event_loop(cls.loop)
        cls.client.__exit__()

    def subscribe_one(self, Message, topic: str) -> Future:
        """
        Returns a future that is set when the first subscription message is received.
        Need to call "spin_until" to start the subscription.
        """
        fut = Future()

        def on_msg(msg):
            self.node.destroy_subscription(sub)
            fut.set_result(msg)

        sub = self.node.create_subscription(Message, topic, on_msg, 1)
        return fut

    def host_service_one(self, Service, srv_name: str, response):
        """
        Hosts a service until a request is received. Returns a future that is set when
        the first request is received. The node is spun in a background thread.
        """
        ros_fut = rclpy.task.Future(executor=self.rcl_executor)
        loop = asyncio.get_event_loop()
        fut = asyncio.Future()

        def on_request(request, _resp):
            ros_fut.set_result(request)
            return response

        srv = self.node.create_service(Service, srv_name, on_request)

        def spin():
            rclpy.spin_until_future_complete(self.node, ros_fut, self.rcl_executor, 1)
            self.node.destroy_service(srv)
            loop.call_soon_threadsafe(fut.set_result, ros_fut.result())

        Thread(target=spin).start()
        cli = self.node.create_client(Service, srv_name)
        if not cli.wait_for_service(1):
            raise RuntimeError("fail to create service")
        return fut

    def spin_until(self, fut: rclpy.task.Future):
        rclpy.spin_until_future_complete(self.node, fut, self.rcl_executor)
        return fut.result()
