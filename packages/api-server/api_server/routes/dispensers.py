from typing import List, cast

from fastapi import Depends
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import Dispenser, DispenserHealth, DispenserState
from api_server.repositories import RmfRepository


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

        @self.sub("/{guid}/state", response_model=DispenserState)
        async def sub_dispenser_state(req: SubscriptionRequest, guid: str):
            user = req.session["user"]
            dispenser_state = await get_dispenser_state(guid, RmfRepository(user))
            if dispenser_state is not None:
                await req.sio.emit(req.room, dispenser_state.dict(), req.sid)
            return app.rmf_events().dispenser_states.pipe(
                rxops.filter(lambda x: cast(DispenserState, x).guid == guid)
            )

        @self.get("/{guid}/health", response_model=DispenserHealth)
        async def get_dispenser_health(
            guid: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_dispenser_health(guid)

        @self.sub("/{guid}/health", response_model=DispenserHealth)
        async def sub_dispenser_health(req: SubscriptionRequest, guid: str):
            user = req.session["user"]
            health = await get_dispenser_health(guid, RmfRepository(user))
            if health is not None:
                await req.sio.emit(req.room, health.dict(), req.sid)
            return app.rmf_events().dispenser_health.pipe(
                rxops.filter(lambda x: cast(DispenserHealth, x).id_ == guid)
            )
