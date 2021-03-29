import rclpy.node
from fastapi import APIRouter
from rmf_lift_msgs.msg import LiftRequest as RmfLiftRequest

from ..models import LiftRequest


def lifts_router(ros_node: rclpy.node.Node):
    router = APIRouter()

    @router.post("/{lift_name}/request")
    async def _post_lift_request(lift_name: str, lift_request: LiftRequest):
        msg = RmfLiftRequest(
            lift_name=lift_name,
            request_time=ros_node.get_clock().now().to_msg(),
            session_id=ros_node.get_name(),
            request_type=lift_request.request_type,
            destination_floor=lift_request.destination,
            door_state=lift_request.door_mode,
        )
        ros_node.door_req.publish(msg)

    return router
