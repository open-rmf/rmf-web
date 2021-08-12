from typing import List

from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter, WatchRequest
from api_server.models import Door, DoorHealth, DoorRequest, DoorState
from api_server.repositories import RmfRepository
from fastapi import Depends
from rx import operators as rxops

from .utils import rx_watcher


class DoorsRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Doors"])

        @self.get("", response_model=List[Door])
        async def get_doors(rmf_repo: RmfRepository = Depends(app.rmf_repo)):
            return await rmf_repo.get_doors()

        @self.get("/{door_name}/state", response_model=DoorState)
        async def get_door_state(
            door_name: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_door_state(door_name)

        @self.watch("/{door_name}/state")
        async def watch_door_state(req: WatchRequest, door_name: str):
            door_state = await get_door_state(door_name, RmfRepository(req.user))
            if door_state:
                await req.emit(door_state.dict())
            rx_watcher(
                req,
                app.rmf_events().door_states.pipe(
                    rxops.filter(lambda x: x.door_name == door_name),
                    rxops.map(lambda x: x.dict()),
                ),
            )

        @self.get("/{door_name}/health", response_model=DoorHealth)
        async def get_door_health(
            door_name: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_door_health(door_name)

        @self.watch("/{door_name}/health")
        async def watch_door_health(req: WatchRequest, door_name: str):
            health = await get_door_health(door_name, RmfRepository(req.user))
            await req.emit(health.to_dict())
            rx_watcher(
                req,
                app.rmf_events().door_health.pipe(
                    rxops.filter(lambda x: x.id_ == door_name),
                    rxops.map(lambda x: x.dict()),
                ),
            )

        @self.post("/{door_name}/request")
        def post_door_request(
            door_name: str,
            door_request: DoorRequest,
        ):
            app.rmf_gateway().request_door(door_name, door_request.mode)
