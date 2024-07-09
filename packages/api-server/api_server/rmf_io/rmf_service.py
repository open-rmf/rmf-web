import asyncio
import contextlib
import logging
from asyncio import Future
from uuid import uuid4

import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from fastapi import HTTPException
from rmf_task_msgs.msg import ApiRequest, ApiResponse

from api_server.fast_io import singleton_dep
from api_server.ros import get_ros_node


class RmfService(contextlib.AbstractContextManager):
    """
    RMF uses a pseudo service protocol implmented using pub/sub. "Calling" a service
    involves publishing a request message with a request id and subscribing to a response.

    Any node can response to the request, so responses may come in out of order and there
    is no guarantee that there will be only one response, clients must keep track of the
    request ids which they published and drop and unknown and duplicated response ids.
    """

    def __init__(
        self,
        ros_node: rclpy.node.Node,
        request_topic: str,
        response_topic: str,
    ):
        self.ros_node = ros_node
        self._request_topic = request_topic
        self._response_topic = response_topic
        self._api_pub: rclpy.publisher.Publisher
        self._api_sub: rclpy.subscription.Subscription
        self._requests: dict[str, Future] = {}

    def __enter__(self):
        self._api_pub = self.ros_node.create_publisher(
            ApiRequest,
            self._request_topic,
            rclpy.qos.QoSProfile(
                depth=10,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._api_sub = self.ros_node.create_subscription(
            ApiResponse,
            self._response_topic,
            self._handle_response,
            rclpy.qos.QoSProfile(
                depth=10,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        return self

    def __exit__(self, *exc):
        """
        Unsubscribes to api responses and destroys all ros objects created by this class.
        """
        self._api_sub.destroy()
        self._api_pub.destroy()

    async def call(self, payload: str, timeout: float = 5) -> str:
        req_id = str(uuid4())
        msg = ApiRequest(request_id=req_id, json_msg=payload)
        fut = Future()
        self._requests[req_id] = fut
        self._api_pub.publish(msg)
        logging.info(f"sent request '{req_id}'")
        logging.debug(msg)
        try:
            return await asyncio.wait_for(fut, timeout)
        except asyncio.TimeoutError as e:
            raise HTTPException(500, "rmf service timed out") from e
        finally:
            del self._requests[req_id]

    def _handle_response(self, msg: ApiResponse):
        logging.info(f"got response '{msg.request_id}'")
        logging.debug(msg)
        fut = self._requests.get(msg.request_id)
        if fut is None:
            logging.warning(
                f"Received response for unknown request id: {msg.request_id}"
            )
            return
        fut.set_result(msg.json_msg)


@singleton_dep
def get_tasks_service():
    return RmfService(get_ros_node(), "task_api_requests", "task_api_responses")
