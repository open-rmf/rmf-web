from fastapi import Depends
from rx import operators as rxops

from api_server.base_app import BaseApp
from api_server.dependencies import cache_control
from api_server.fast_io import FastIORouter, WatchRequest
from api_server.models import BuildingMap
from api_server.repositories.rmf import RmfRepository

from .utils import rx_watcher


class BuildingMapRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Building"])

        @self.get(
            "", response_model=BuildingMap, dependencies=[Depends(cache_control())]
        )
        async def get_building_map(rmf_repo: RmfRepository = Depends(app.rmf_repo)):
            """
            Available in socket.io
            """
            return await rmf_repo.get_bulding_map()

        @self.watch("")
        async def watch_building_map(req: WatchRequest):
            rx_watcher(
                req,
                app.rmf_events().building_map.pipe(
                    rxops.filter(lambda x: x is not None),
                    rxops.map(lambda x: x.dict()),
                ),
            )
