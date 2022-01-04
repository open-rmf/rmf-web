import asyncio
from asyncio import Future
from typing import Dict

import rclpy
import rclpy.node
import rclpy.qos
from rmf_task_msgs.msg import ApiRequest, ApiResponse

from api_server.logger import logger


class RmfService:
    """
    RMF uses a pseudo service protocol implmented using pub/sub. "Calling" a service
    involves publishing a request message with a request id and subscribing to a response.

    Any node can response to the request, so responses may come in out of order and there
    is no guarantee that there will be only one response, clients must keep track of the
    request ids which they published and drop and unknown and duplicated response ids.
    """

    API_REQUEST_TOPIC = "api_request"
    API_RESPONSE_TOPIC = "api_response"

    def __init__(self, ros_node: rclpy.node.Node):
        self.ros_node = ros_node
        self._logger = logger.getChild(self.__class__.__name__)
        self._id_counter = 0
        self._requests: Dict[str, Future] = {}
        self._api_pub = self.ros_node.create_publisher(
            ApiRequest,
            self.API_REQUEST_TOPIC,
            rclpy.qos.QoSProfile(
                depth=10,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._api_sub = self.ros_node.create_subscription(
            ApiResponse,
            self.API_RESPONSE_TOPIC,
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

    async def call(self, payload: str, timeout=1) -> str:
        req_id = str(self._id_counter)
        self._id_counter += 1
        msg = ApiRequest(request_id=req_id, json_msg=payload)
        fut = Future()
        self._requests[req_id] = fut
        self._api_pub.publish(msg)
        resp = await asyncio.wait_for(fut, timeout)
        del self._requests[req_id]
        return resp

    def _handle_response(self, msg: ApiResponse):
        fut = self._requests.get(msg.request_id)
        if fut is None:
            self._logger.warning(
                f"Received response for unknown request id: {msg.request_id}"
            )
            return
        fut.set_result(msg.json_msg)
