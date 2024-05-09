from api_server.rmf_io import rmf_events
from api_server.test import (
    AppFixture,
    make_building_map,
    make_fire_alarm_trigger,
    try_until,
)


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

    def test_get_previous_fire_alarm_trigger(self):
        true_trigger = make_fire_alarm_trigger(True)
        rmf_events.fire_alarm_trigger.on_next(true_trigger)
        resp = self.client.get("/building_map/previous_fire_alarm_trigger")
        self.assertEqual(200, resp.status_code)
        result = resp.json()
        self.assertTrue(result["trigger"])

        false_trigger = make_fire_alarm_trigger(False)
        rmf_events.fire_alarm_trigger.on_next(false_trigger)
        resp = self.client.get("/building_map/previous_fire_alarm_trigger")
        self.assertEqual(200, resp.status_code)
        result = resp.json()
        self.assertFalse(result["trigger"])

    def test_reset_fire_alarm_trigger(self):
        true_trigger = make_fire_alarm_trigger(True)
        rmf_events.fire_alarm_trigger.on_next(true_trigger)
        resp = self.client.get("/building_map/previous_fire_alarm_trigger")
        self.assertEqual(200, resp.status_code)
        result = resp.json()
        self.assertTrue(result["trigger"])

        resp = self.client.post("/building_map/reset_fire_alarm_trigger")
        self.assertEqual(200, resp.status_code)
        result = resp.json()
        self.assertFalse(result["trigger"])
