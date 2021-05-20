from typing import List

from ..fast_io import FastIORouter
from ..models import Dispenser, DispenserHealth, DispenserState
from ..repositories import RmfRepository
from ..rmf_io import RmfEvents


class DispensersRouter(FastIORouter):
    def __init__(
        self,
        rmf_events: RmfEvents,
        rmf_repo: RmfRepository,
    ):
        super().__init__(tags=["Dispensers"])

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
