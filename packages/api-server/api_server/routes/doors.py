from typing import Callable, List

from fastapi import Depends
from rmf_door_msgs.msg import DoorMode as RmfDoorMode
from rmf_door_msgs.msg import DoorRequest as RmfDoorRequest

from ..fast_io import FastIORouter
from ..gateway import RmfGateway
from ..models import Door, DoorHealth, DoorRequest, DoorState
from ..repositories import RmfRepository
from ..rmf_io import RmfEvents


class DoorsRouter(FastIORouter):
    def __init__(
        self,
        rmf_events: RmfEvents,
        rmf_gateway_dep: Callable[[], RmfGateway],
        rmf_repo: RmfRepository,
    ):
        super().__init__(tags=["Doors"])

        @self.get("", response_model=List[Door])
        async def get_doors():
            return await rmf_repo.get_doors()

        @self.watch(
            "/{door_name}/state", rmf_events.door_states, response_model=DoorState
        )
        def get_door_state(door_state: DoorState):
            return {"door_name": door_state.door_name}, door_state

        @self.watch(
            "/{door_name}/health", rmf_events.door_health, response_model=DoorHealth
        )
        def get_door_health(door_health: DoorHealth):
            return {"door_name": door_health.id_}, door_health

        @self.post("/{door_name}/request")
        async def post_door_request(
            door_name: str,
            door_request: DoorRequest,
            ros_node: RmfGateway = Depends(rmf_gateway_dep),
        ):
            msg = RmfDoorRequest(
                door_name=door_name,
                request_time=ros_node.get_clock().now().to_msg(),
                requester_id=ros_node.get_name(),
                requested_mode=RmfDoorMode(
                    value=door_request.mode,
                ),
            )
            ros_node.door_req.publish(msg)
