from typing import Callable, List

from fastapi import Depends
from rmf_lift_msgs.msg import LiftRequest as RmfLiftRequest

from ..fast_io import FastIORouter
from ..gateway import RmfGateway
from ..models import Lift, LiftHealth, LiftRequest, LiftState
from ..repositories import RmfRepository
from ..rmf_io import RmfEvents


class LiftsRouter(FastIORouter):
    def __init__(
        self,
        rmf_events: RmfEvents,
        rmf_gateway_dep: Callable[[], RmfGateway],
        rmf_repo: RmfRepository,
    ):
        super().__init__(tags=["Lifts"])

        @self.get("", response_model=List[Lift])
        async def get_lifts():
            return await rmf_repo.get_lifts()

        @self.watch(
            "/{lift_name}/state", rmf_events.lift_states, response_model=LiftState
        )
        def get_lift_state(lift_state: LiftState):
            return {"lift_name": lift_state.lift_name}, lift_state

        @self.watch(
            "/{lift_name}/health",
            rmf_events.lift_health,
            response_model=LiftHealth,
        )
        def get_lift_health(lift_health: LiftHealth):
            return {"lift_name": lift_health.id_}, lift_health

        @self.post("/{lift_name}/request")
        async def _post_lift_request(
            lift_name: str,
            lift_request: LiftRequest,
            ros_node: RmfGateway = Depends(rmf_gateway_dep),
        ):
            msg = RmfLiftRequest(
                lift_name=lift_name,
                request_time=ros_node.get_clock().now().to_msg(),
                session_id=ros_node.get_name(),
                request_type=lift_request.request_type,
                destination_floor=lift_request.destination,
                door_state=lift_request.door_mode,
            )
            ros_node.lift_req.publish(msg)
