from typing import List

from api_server.fast_io import FastIORouter
from api_server.models import Ingestor, IngestorHealth, IngestorState
from api_server.repositories import RmfRepository
from api_server.rmf_io import RmfEvents


class IngestorsRouter(FastIORouter):
    def __init__(
        self,
        rmf_events: RmfEvents,
        rmf_repo: RmfRepository,
    ):
        super().__init__(tags=["Ingestors"])

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
