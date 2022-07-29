from api_server.rmf_io import rmf_events
from api_server.test import AppFixture, make_building_map, try_until


class TestBuildingMapRoute(AppFixture):
    def test_get_building_map(self):
        building_map = make_building_map()
        rmf_events.building_map.on_next(building_map)

        resp = try_until(
            lambda: self.client.get("/building_map"), lambda x: x.status_code == 200
        )
        self.assertEqual(200, resp.status_code)
        result_map = resp.json()
        self.assertEqual(building_map.name, result_map["name"])
