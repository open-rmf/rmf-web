from typing import List, cast

from fastapi import Depends
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import Door, DoorHealth, DoorRequest, DoorState
from api_server.repositories import RmfRepository


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

        @self.sub("/{door_name}/state", response_model=DoorState)
        async def sub_door_state(req: SubscriptionRequest, door_name: str):
            user = req.session["user"]
            door_state = await get_door_state(door_name, RmfRepository(user))
            if door_state:
                await req.sio.emit(req.room, door_state.dict(), req.sid)
            return app.rmf_events().door_states.pipe(
                rxops.filter(lambda x: cast(DoorState, x).door_name == door_name)
            )

        @self.get("/{door_name}/health", response_model=DoorHealth)
        async def get_door_health(
            door_name: str, rmf_repo: RmfRepository = Depends(app.rmf_repo)
        ):
            """
            Available in socket.io
            """
            return await rmf_repo.get_door_health(door_name)

        @self.sub("/{door_name}/health", response_model=DoorHealth)
        async def sub_door_health(req: SubscriptionRequest, door_name: str):
            user = req.session["user"]
            health = await get_door_health(door_name, RmfRepository(user))
            if health is not None:
                await req.sio.emit(req.room, health.dict(), req.sid)
            return app.rmf_events().door_health.pipe(
                rxops.filter(lambda x: cast(DoorHealth, x).id_ == door_name)
            )

        @self.post("/{door_name}/request")
        def post_door_request(
            door_name: str,
            door_request: DoorRequest,
        ):
            app.rmf_gateway().request_door(door_name, door_request.mode)
