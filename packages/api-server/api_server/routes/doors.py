import rclpy.node
from fastapi import APIRouter
from rmf_door_msgs.msg import DoorMode as RmfDoorMode
from rmf_door_msgs.msg import DoorRequest as RmfDoorRequest

from ..models import DoorMode


def doors_router(ros_node: rclpy.node.Node):
    router = APIRouter()

    @router.post("/{door_name}/request")
    async def _post_door_request(door_name: str, mode: DoorMode):
        msg = RmfDoorRequest(
            door_name=door_name,
            request_time=ros_node.get_clock().now().to_msg(),
            requester_id=ros_node.get_name(),
            requested_mode=RmfDoorMode(
                value=mode.mode,
            ),
        )
        ros_node.door_req.publish(msg)

    return router
