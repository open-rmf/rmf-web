from typing import List

from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter
from api_server.models import Ingestor, IngestorHealth, IngestorState


class IngestorsRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Ingestors"])
        rmf_repo = app.rmf_repo
        rmf_events = app.rmf_events

        @self.get("", response_model=List[Ingestor])
        async def get_ingestors():
            return await rmf_repo.query_ingestors()

        @self.watch(
            "/{guid}/state", rmf_events.ingestor_states, response_model=IngestorState
        )
        def get_ingestor_state(ingestor_state: IngestorState):
            return {"guid": ingestor_state.guid}, ingestor_state

        @self.watch(
            "/{guid}/health",
            rmf_events.ingestor_health,
            response_model=IngestorHealth,
        )
        def get_ingestor_health(ingestor_health: IngestorHealth):
            return {"guid": ingestor_health.id_}, ingestor_health
