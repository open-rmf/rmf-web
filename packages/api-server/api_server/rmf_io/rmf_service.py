import asyncio
from asyncio import Future
from typing import Callable, Dict, Optional
from uuid import uuid4

import rclpy
import rclpy.node
import rclpy.qos
from fastapi import HTTPException
from rmf_task_msgs.msg import ApiRequest, ApiResponse

from api_server.logger import logger
from api_server.ros import ros_node as default_ros_node


class RmfService:
    """
    RMF uses a pseudo service protocol implmented using pub/sub. "Calling" a service
    involves publishing a request message with a request id and subscribing to a response.

    Any node can response to the request, so responses may come in out of order and there
    is no guarantee that there will be only one response, clients must keep track of the
    request ids which they published and drop and unknown and duplicated response ids.
    """

    def __init__(
        self,
        ros_node: Callable[[], rclpy.node.Node],
        request_topic: str,
        response_topic: str,
    ):
        self.ros_node = ros_node
        self._logger = logger.getChild(self.__class__.__name__)
        self._requests: Dict[str, Future] = {}
        self._api_pub = self.ros_node().create_publisher(
            ApiRequest,
            request_topic,
            rclpy.qos.QoSProfile(
                depth=10,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._api_sub = self.ros_node().create_subscription(
            ApiResponse,
            response_topic,
            self._handle_response,
            rclpy.qos.QoSProfile(
                depth=10,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def destroy(self):
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
        self._logger.info(f"sent request '{req_id}'")
        self._logger.debug(msg)
        try:
            return await asyncio.wait_for(fut, timeout)
        except asyncio.TimeoutError as e:
            raise HTTPException(500, "rmf service timed out") from e
        finally:
            del self._requests[req_id]

    def _handle_response(self, msg: ApiResponse):
        self._logger.info(f"got response '{msg.request_id}'")
        self._logger.debug(msg)
        fut = self._requests.get(msg.request_id)
        if fut is None:
            self._logger.warning(
                f"Received response for unknown request id: {msg.request_id}"
            )
            return
        fut.set_result(msg.json_msg)


_tasks_service: Optional[RmfService] = None


def tasks_service():
    global _tasks_service
    if _tasks_service is None:
        _tasks_service = RmfService(
            default_ros_node, "task_api_requests", "task_api_responses"
        )
    return _tasks_service
