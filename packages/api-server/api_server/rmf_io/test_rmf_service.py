import asyncio
import threading
import time
import unittest
from uuid import uuid4

import rclpy
import rclpy.context
import rclpy.executors
import rclpy.node
import rclpy.qos
from rmf_task_msgs.msg import ApiRequest, ApiResponse

from .rmf_service import RmfService


class TestRmfService(unittest.TestCase):
    @staticmethod
    def _create_node_id() -> str:
        return str(uuid4()).replace("-", "_")

    def setUp(self) -> None:
        self._client_context = rclpy.context.Context()
        self._client_context.init()
        self.client_node = rclpy.node.Node(
            f"test_client{self._create_node_id()}", context=self._client_context
        )
        self._client_executor = rclpy.executors.SingleThreadedExecutor()

        def client():
            while self._client_context.ok():
                rclpy.spin_once(
                    self.client_node, executor=self._client_executor, timeout_sec=0.1
                )

        self._client_thread = threading.Thread(target=client)
        self._client_thread.start()

        self._server_context = rclpy.context.Context()
        self._server_context.init()
        self.server_node = rclpy.node.Node(
            f"test_server_{self._create_node_id()}",
            context=self._server_context,
        )
        self._server_executor = rclpy.executors.SingleThreadedExecutor()

        def server():
            pub = self.server_node.create_publisher(
                ApiResponse,
                RmfService.API_RESPONSE_TOPIC,
                rclpy.qos.QoSProfile(
                    depth=10,
                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )

            def handle_resp(msg: ApiRequest):
                pub.publish(
                    ApiResponse(request_id=msg.request_id, json_msg=msg.json_msg)
                )

            self.server_node.create_subscription(
                ApiRequest,
                RmfService.API_REQUEST_TOPIC,
                handle_resp,
                rclpy.qos.QoSProfile(
                    depth=10,
                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )

            while self._server_context.ok():
                rclpy.spin_once(
                    self.server_node, executor=self._server_executor, timeout_sec=0.1
                )

        self._server_thread = threading.Thread(target=server)
        self._server_thread.start()

        # wait for discovery
        while self.server_node.get_name() not in self.client_node.get_node_names():
            time.sleep(0.1)
        while self.client_node.get_name() not in self.server_node.get_node_names():
            time.sleep(0.1)

        self.rmf_service = RmfService(self.client_node)

    def tearDown(self) -> None:
        self._client_context.shutdown()
        self._client_executor.shutdown()
        self._client_thread.join()
        self._server_context.shutdown()
        self._server_executor.shutdown()
        self._server_thread.join()

    def test_call(self):
        async def run():
            result = await self.rmf_service.call("hello")
            self.assertEqual("hello", result)

        asyncio.get_event_loop().run_until_complete(run())

    def test_multiple_calls(self):
        async def run():
            tasks = [self.rmf_service.call("hello"), self.rmf_service.call("world")]
            results = await asyncio.gather(*tasks)
            self.assertEqual("hello", results[0])
            self.assertEqual("world", results[1])  # type: ignore (bug in pylance)

        asyncio.get_event_loop().run_until_complete(run())
