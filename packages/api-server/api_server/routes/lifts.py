from fastapi import APIRouter
from rmf_lift_msgs.msg import LiftRequest as RmfLiftRequest

from ..dependencies import ros
from ..models import LiftRequest

router = APIRouter(tags=["lifts"])


@router.post("/{lift_name}/request")
async def _post_lift_request(lift_name: str, lift_request: LiftRequest):
    msg = RmfLiftRequest(
        lift_name=lift_name,
        request_time=ros.node.get_clock().now().to_msg(),
        session_id=ros.node.get_name(),
        request_type=lift_request.request_type,
        destination_floor=lift_request.destination,
        door_state=lift_request.door_mode,
    )
    ros.node.lift_req.publish(msg)
