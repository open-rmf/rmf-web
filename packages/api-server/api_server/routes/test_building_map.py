from api_server.test import AppFixture, make_building_map


class TestBuildingMapRoute(AppFixture):
    async def test_get_building_map(self):
        building_map = make_building_map()
        self.app.rmf_events().building_map.on_next(building_map)

        resp = await self.client.get("/building_map")
        self.assertEqual(200, resp.status_code)
        result_map = resp.json()
        self.assertEqual(building_map.name, result_map["name"])
