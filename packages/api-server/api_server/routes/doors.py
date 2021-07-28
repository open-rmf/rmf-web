from typing import List

from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter
from api_server.models import Door, DoorHealth, DoorRequest, DoorState


class DoorsRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Doors"])
        rmf_events = app.rmf_events

        @self.get("", response_model=List[Door])
        async def get_doors():
            return await app.rmf_repo.get_doors()

        @self.watch(
            "/{door_name}/state", rmf_events.door_states, response_model=DoorState
        )
        def get_door_state(door_state: DoorState):
            return {"door_name": door_state.door_name}, door_state

        @self.watch(
            "/{door_name}/health", rmf_events.door_health, response_model=DoorHealth
        )
        def get_door_health(door_health: DoorHealth):
            return {"door_name": door_health.id_}, door_health

        @self.post("/{door_name}/request")
        def post_door_request(
            door_name: str,
            door_request: DoorRequest,
        ):
            app.rmf_gateway.request_door(door_name, door_request.mode)
