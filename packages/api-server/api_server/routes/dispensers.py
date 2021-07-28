from typing import List

from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter
from api_server.models import Dispenser, DispenserHealth, DispenserState


class DispensersRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Dispensers"])
        rmf_repo = app.rmf_repo
        rmf_events = app.rmf_events

        @self.get("", response_model=List[Dispenser])
        async def get_dispensers():
            return await rmf_repo.query_dispensers()

        @self.watch(
            "/{guid}/state", rmf_events.dispenser_states, response_model=DispenserState
        )
        def get_dispenser_state(dispenser_state: DispenserState):
            return {"guid": dispenser_state.guid}, dispenser_state

        @self.watch(
            "/{guid}/health",
            rmf_events.dispenser_health,
            response_model=DispenserHealth,
        )
        def get_dispenser_health(dispenser_health: DispenserHealth):
            return {"guid": dispenser_health.id_}, dispenser_health
