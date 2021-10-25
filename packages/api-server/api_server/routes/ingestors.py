from typing import List

from fastapi import Depends
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.dependencies import cache_control
from api_server.fast_io import FastIORouter, WatchRequest
from api_server.models import Ingestor, IngestorHealth, IngestorState
from api_server.repositories import RmfRepository

from .utils import rx_watcher


class IngestorsRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Ingestors"])

        @self.get(
            "", response_model=List[Ingestor], dependencies=[Depends(cache_control())]
        )
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

        @self.watch("/{guid}/state")
        async def watch_ingestor_state(req: WatchRequest, guid: str):
            ingestor_state = await get_ingestor_state(guid, RmfRepository(req.user))
            await req.emit(ingestor_state.dict())
            rx_watcher(
                req,
                app.rmf_events().ingestor_states.pipe(
                    rxops.filter(lambda x: x.guid == guid),
                    rxops.map(lambda x: x.dict()),
                ),
            )

        @self.get("/{guid}/health", response_model=IngestorHealth)
        async def get_ingestor_health(
            guid: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_ingestor_health(guid)

        @self.watch("/{guid}/health")
        async def watch_ingestor_health(req: WatchRequest, guid: str):
            health = await get_ingestor_health(guid, RmfRepository(req.user))
            await req.emit(health.dict())
            rx_watcher(
                req,
                app.rmf_events().ingestor_health.pipe(
                    rxops.filter(lambda x: x.id_ == guid),
                    rxops.map(lambda x: x.dict()),
                ),
            )
