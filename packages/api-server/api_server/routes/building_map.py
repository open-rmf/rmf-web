from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter, WatchRequest
from api_server.models import BuildingMap
from api_server.routes.utils import rx_watcher


class BuildingMapRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Building"])

        @self.get("", response_model=BuildingMap)
        async def get_building_map():
            """
            Available in socket.io
            """
            return await app.rmf_repo.get_bulding_map()

        @self.watch("")
        async def watch_building_map(req: WatchRequest):
            rx_watcher(
                req,
                app.rmf_events().building_map.pipe(
                    rxops.filter(lambda x: x is not None)
                ),
            )
