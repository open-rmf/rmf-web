from typing import List, cast

from fastapi import Depends
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import Ingestor, IngestorHealth, IngestorState
from api_server.repositories import RmfRepository


class IngestorsRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Ingestors"])

        @self.get("", response_model=List[Ingestor])
        async def get_ingestors(rmf_repo: RmfRepository = Depends(app.rmf_repo)):
            return await rmf_repo.get_ingestors()

        @self.get("/{guid}/state", response_model=IngestorState)
        async def get_ingestor_state(
            guid: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_ingestor_state(guid)

        @self.sub("/{guid}/state", response_model=IngestorState)
        async def sub_ingestor_state(req: SubscriptionRequest, guid: str):
            user = req.session["user"]
            ingestor_state = await get_ingestor_state(guid, RmfRepository(user))
            if ingestor_state is not None:
                await req.sio.emit(req.room, ingestor_state.dict(), req.sid)
            return app.rmf_events().ingestor_states.pipe(
                rxops.filter(lambda x: cast(IngestorState, x).guid == guid)
            )

        @self.get("/{guid}/health", response_model=IngestorHealth)
        async def get_ingestor_health(
            guid: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_ingestor_health(guid)

        @self.sub("/{guid}/health", response_model=IngestorHealth)
        async def sub_ingestor_health(req: SubscriptionRequest, guid: str):
            user = req.session["user"]
            health = await get_ingestor_health(guid, RmfRepository(user))
            if health is not None:
                await req.sio.emit(req.room, health.dict(), req.sid)
            return app.rmf_events().ingestor_health.pipe(
                rxops.filter(lambda x: cast(IngestorHealth, x).id_ == guid)
            )
