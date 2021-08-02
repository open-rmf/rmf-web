from typing import List

from fastapi import Depends
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter, WatchRequest
from api_server.models import Dispenser, DispenserHealth, DispenserState
from api_server.repositories import RmfRepository

from .utils import rx_watcher


class DispensersRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Dispensers"])

        @self.get("", response_model=List[Dispenser])
        async def get_dispensers(rmf_repo: RmfRepository = Depends(app.rmf_repo)):
            return await rmf_repo.get_dispensers()

        @self.get("/{guid}/state", response_model=DispenserState)
        async def get_dispenser_state(
            guid: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_dispenser_state(guid)

        @self.watch("/{guid}/state")
        async def watch_dispenser_state(req: WatchRequest, guid: str):
            dispenser_state = await get_dispenser_state(guid, RmfRepository(req.user))
            await req.emit(dispenser_state.dict())
            rx_watcher(
                req,
                app.rmf_events().dispenser_states.pipe(
                    rxops.filter(lambda x: x.guid == guid)
                ),
            )

        @self.get("/{guid}/health", response_model=DispenserHealth)
        async def get_dispenser_health(
            guid: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_dispenser_health(guid)

        @self.watch("/{guid}/health")
        async def watch_dispenser_health(req: WatchRequest, guid: str):
            health = await get_dispenser_health(guid, RmfRepository(req.user))
            await req.emit(health.dict())
            rx_watcher(
                req,
                app.rmf_events().dispenser_health.pipe(
                    rxops.filter(lambda x: x.id_ == guid)
                ),
            )
