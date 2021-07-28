from api_server.base_app import BaseApp
from api_server.fast_io import FastIORouter
from api_server.models import BuildingMap


class BuildingMapRouter(FastIORouter):
    def __init__(self, app: BaseApp):
        super().__init__(tags=["Building"])

        @self.watch("", app.rmf_events.building_map, response_model=BuildingMap)
        def watch_building_map(building_map: BuildingMap):
            if building_map is None:
                return None
            return {}, building_map
