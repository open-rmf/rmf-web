import os
import os.path
import threading
import time
import unittest
from concurrent.futures import Future

import rclpy
import rclpy.node
import requests
from requests.adapters import Response
from urllib3.util.retry import Retry

from ..app import App
from ..app_config import load_config
from ..test.server import BackgroundServer


class RouteFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = App(load_config(f"{os.path.dirname(__file__)}/test_config.py"))
        cls.server = BackgroundServer(cls.app)
        cls.server.start()
        cls.base_url = cls.server.base_url

        retry = Retry(total=5, backoff_factor=0.1)
        adapter = requests.adapters.HTTPAdapter(max_retries=retry)
        cls.session = requests.Session()
        cls.session.mount("http://", adapter)

        cls.rcl_ctx = rclpy.Context()
        rclpy.init(context=cls.rcl_ctx)
        cls.rcl_executor = rclpy.executors.SingleThreadedExecutor(context=cls.rcl_ctx)
        cls.node = rclpy.node.Node("test_node", context=cls.rcl_ctx)

        tries = 0
        while "rmf_api_server" not in cls.node.get_node_names():
            tries += 1
            if tries >= 10:
                raise TimeoutError("cannot discover rmf_api_server node")
            time.sleep(0.5)
        # give ros more time, tests are very flaky when discovery is not finished
        time.sleep(1)

    @classmethod
    def tearDownClass(cls):
        cls.session.close()

        cls.node.destroy_node()
        rclpy.shutdown(context=cls.rcl_ctx)

        cls.server.stop()

    def try_get(self, url: str, expected_response=200, timeout=5, interval=0.5):
        """
        Do GET requests until an expected response code is received.
        Returns the last response received.
        """
        end_time = time.time() + timeout
        resp: Response
        while time.time() < end_time:
            resp = self.session.get(url)
            if resp == expected_response:
                return resp
            time.sleep(interval)
        return resp

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
        fut = Future()

        def on_request(request, _resp):
            fut.set_result(request)
            return response

        srv = self.node.create_service(Service, srv_name, on_request)

        def spin():
            rclpy.spin_until_future_complete(self.node, fut, self.rcl_executor, 1)
            self.node.destroy_service(srv)

        threading.Thread(target=spin).start()
        return fut

    def spin_until(self, fut: rclpy.task.Future, timeout: float):
        rclpy.spin_until_future_complete(
            self.node, fut, self.rcl_executor, timeout_sec=timeout
        )
        return fut.result(0)
