from typing import List, cast

from fastapi import Depends
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import Lift, LiftHealth, LiftRequest, LiftState
from api_server.repositories import RmfRepository


class LiftsRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Lifts"])

        @self.get("", response_model=List[Lift])
        async def get_lifts(rmf_repo: RmfRepository = Depends(app.rmf_repo)):
            return await rmf_repo.get_lifts()

        @self.get("/{lift_name}/state", response_model=LiftState)
        async def get_lift_state(
            lift_name: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_lift_state(lift_name)

        @self.sub("/{lift_name}/state", response_model=LiftState)
        async def sub_lift_state(req: SubscriptionRequest, lift_name: str):
            user = req.session["user"]
            lift_state = await get_lift_state(lift_name, RmfRepository(user))
            if lift_state is not None:
                await req.sio.emit(req.room, lift_state.dict(), req.sid)
            return app.rmf_events().lift_states.pipe(
                rxops.filter(lambda x: cast(LiftState, x).lift_name == lift_name)
            )

        @self.get("/{lift_name}/health", response_model=LiftHealth)
        async def get_lift_health(
            lift_name: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_lift_health(lift_name)

        @self.sub("/{lift_name}/health", response_model=LiftHealth)
        async def sub_lift_health(req: SubscriptionRequest, lift_name: str):
            user = req.session["user"]
            health = await get_lift_health(lift_name, RmfRepository(user))
            if health is not None:
                await req.sio.emit(req.room, health.dict(), req.sid)
            return app.rmf_events().lift_health.pipe(
                rxops.filter(lambda x: cast(LiftHealth, x).id_ == lift_name),
            )

        @self.post("/{lift_name}/request")
        def _post_lift_request(
            lift_name: str,
            lift_request: LiftRequest,
        ):
            app.rmf_gateway().request_lift(
                lift_name,
                lift_request.destination,
                lift_request.request_type,
                lift_request.door_mode,
            )
