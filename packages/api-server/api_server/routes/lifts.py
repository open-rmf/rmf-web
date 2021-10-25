from typing import List

from fastapi import Depends
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.dependencies import cache_control
from api_server.fast_io import FastIORouter, WatchRequest
from api_server.models import Lift, LiftHealth, LiftRequest, LiftState
from api_server.repositories import RmfRepository

from .utils import rx_watcher


class LiftsRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Lifts"])

        @self.get(
            "", response_model=List[Lift], dependencies=[Depends(cache_control())]
        )
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

        @self.watch("/{lift_name}/state")
        async def watch_lift_state(req: WatchRequest, lift_name: str):
            lift_state = await get_lift_state(lift_name, RmfRepository(req.user))
            if lift_state is not None:
                await req.emit(lift_state.dict())
            rx_watcher(
                req,
                app.rmf_events().lift_states.pipe(
                    rxops.filter(lambda x: x.lift_name == lift_name),
                    rxops.map(lambda x: x.dict()),
                ),
            )

        @self.get("/{lift_name}/health", response_model=LiftHealth)
        async def get_lift_health(
            lift_name: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_lift_health(lift_name)

        @self.watch("/{lift_name}/health")
        async def watch_lift_health(req: WatchRequest, lift_name: str):
            health = await get_lift_health(lift_name, RmfRepository(req.user))
            await req.emit(health.dict())
            rx_watcher(
                req,
                app.rmf_events().lift_health.pipe(
                    rxops.filter(lambda x: x.id_ == lift_name),
                    rxops.map(lambda x: x.dict()),
                ),
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
