from typing import List

from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter
from api_server.models import Lift, LiftHealth, LiftRequest, LiftState


class LiftsRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Lifts"])
        rmf_repo = app.rmf_repo
        rmf_events = app.rmf_events

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
        def _post_lift_request(
            lift_name: str,
            lift_request: LiftRequest,
        ):
            app.rmf_gateway.request_lift(
                lift_name,
                lift_request.destination,
                lift_request.request_type,
                lift_request.door_mode,
            )
