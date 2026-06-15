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

    @classmethod
    def setUpClass(cls) -> None:
        cls._client_context = rclpy.context.Context()
        cls._client_context.init()
        cls.client_node = rclpy.node.Node(
            f"test_client{cls._create_node_id()}", context=cls._client_context
        )  # type: ignore rclpy has invalid typing
        cls._client_executor = rclpy.executors.SingleThreadedExecutor(
            context=cls._client_context
        )

        def client():
            while cls._client_context.ok():
                rclpy.spin_once(
                    cls.client_node, executor=cls._client_executor, timeout_sec=0.1
                )

        cls._client_thread = threading.Thread(target=client)
        cls._client_thread.start()

        cls._server_context = rclpy.context.Context()
        cls._server_context.init()
        cls.server_node = rclpy.node.Node(
            f"test_server_{cls._create_node_id()}",
            context=cls._server_context,
        )  # type: ignore rclpy has invalid typing
        cls._server_executor = rclpy.executors.SingleThreadedExecutor(
            context=cls._server_context
        )

        def server():
            pub = cls.server_node.create_publisher(
                ApiResponse,
                "test_response",
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

            cls.server_node.create_subscription(
                ApiRequest,
                "test_request",
                handle_resp,
                rclpy.qos.QoSProfile(
                    depth=10,
                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )

            while cls._server_context.ok():
                rclpy.spin_once(
                    cls.server_node, executor=cls._server_executor, timeout_sec=0.1
                )

        cls._server_thread = threading.Thread(target=server)
        cls._server_thread.start()

        # wait for discovery
        while cls.server_node.get_name() not in cls.client_node.get_node_names():
            time.sleep(0.1)
        while cls.client_node.get_name() not in cls.server_node.get_node_names():
            time.sleep(0.1)

        cls.rmf_service = RmfService(
            lambda: cls.client_node, "test_request", "test_response"
        )

        cls.loop = asyncio.new_event_loop()

    @classmethod
    def tearDownClass(cls) -> None:
        cls._client_context.shutdown()
        cls._client_executor.shutdown()
        cls._client_thread.join()
        cls._server_context.shutdown()
        cls._server_executor.shutdown()
        cls._server_thread.join()
        cls.loop.close()

    def test_call(self):
        async def run():
            result = await self.rmf_service.call("hello")
            self.assertEqual("hello", result)

        self.loop.run_until_complete(run())

    def test_multiple_calls(self):
        async def run():
            tasks = [self.rmf_service.call("hello"), self.rmf_service.call("world")]
            results = await asyncio.gather(*tasks)
            self.assertListEqual(["hello", "world"], list(results))

        self.loop.run_until_complete(run())

    def test_duplicate_response_is_ignored(self):
        # Regression: RMF's request/response is a pub/sub pseudo-service, so
        # more than one node may answer the same request_id. A duplicate
        # ApiResponse for an already-resolved future used to raise
        # asyncio.InvalidStateError inside the subscription callback, which runs
        # in the rclpy spin thread and would terminate it -- silently breaking
        # every later task-api call. The handler must drop the duplicate.
        req_id = self._create_node_id()
        fut: asyncio.Future = self.loop.create_future()
        self.rmf_service._requests[req_id] = fut
        try:
            self.rmf_service._handle_response(
                ApiResponse(request_id=req_id, json_msg="first")
            )
            # second, duplicate delivery of the same request_id must not raise
            self.rmf_service._handle_response(
                ApiResponse(request_id=req_id, json_msg="second")
            )
        finally:
            self.rmf_service._requests.pop(req_id, None)
        self.assertTrue(fut.done())
        self.assertEqual("first", fut.result())
