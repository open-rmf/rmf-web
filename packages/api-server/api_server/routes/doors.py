from fastapi import APIRouter
from rmf_door_msgs.msg import DoorMode as RmfDoorMode
from rmf_door_msgs.msg import DoorRequest as RmfDoorRequest

from ..dependencies import ros
from ..models import DoorRequest

router = APIRouter()


@router.post("/{door_name}/request")
async def _post_door_request(door_name: str, door_request: DoorRequest):
    msg = RmfDoorRequest(
        door_name=door_name,
        request_time=ros.node.get_clock().now().to_msg(),
        requester_id=ros.node.get_name(),
        requested_mode=RmfDoorMode(
            value=door_request.mode,
        ),
    )
    ros.node.door_req.publish(msg)
