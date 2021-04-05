from ..dependencies.ros import rmf_gateway
from ..rmf_io.test_data import make_building_map
from .test_fixtures import RouteFixture


class TestBuildingMapRoute(RouteFixture):
    def test_get_building_map(self):
        resp = self.client.get("/building_map")
        self.assertEqual(resp.status_code, 503)
        rmf_gateway.rmf_building_map.on_next(make_building_map())
        resp = self.client.get("/building_map")
        self.assertEqual(resp.status_code, 200)
