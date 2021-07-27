from api_server.fast_io import FastIORouter
from api_server.models import BuildingMap
from api_server.rmf_io import RmfEvents


class BuildingMapRouter(FastIORouter):
    def __init__(
        self,
        rmf_events: RmfEvents,
    ):
        super().__init__(tags=["Building"])

        @self.watch("", rmf_events.building_map, response_model=BuildingMap)
        def watch_building_map(building_map: BuildingMap):
            if building_map is None:
                return None
            return {}, building_map
