import unittest

from models.fleet_state import FleetState

from .fleet_state_parser import fleet_state_parser

# fleet_state:{"name": "tinyRobot", "robots": [{"name": "tinyRobot1", "model": "", "task_id": "", "seq": 3190, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1598, "nanosec": 184999999}, "x": 11.553672790527344, "y": -11.317496299743652, "yaw": -1.599777340888977, "level_name": "L1", "index": 0}, "path": []}, {"name": "tinyRobot2", "model": "", "task_id": "", "seq": 3191, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1598, "nanosec": 685999999}, "x": 15.157517433166504, "y": -11.228611946105957, "yaw": -1.5839587450027466, "level_name": "L1", "index": 0}, "path": []}]}\n',


class TestCaseFleetState(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.data = 'fleet_state:{"name": "tinyRobot", "robots": [{"name": "tinyRobot1", "model": "", "task_id": "", "seq": 3190, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1598, "nanosec": 184999999}, "x": 11.553672790527344, "y": -11.317496299743652, "yaw": -1.599777340888977, "level_name": "L1", "index": 0}, "path": []}, {"name": "tinyRobot2", "model": "", "task_id": "", "seq": 3191, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1598, "nanosec": 685999999}, "x": 15.157517433166504, "y": -11.228611946105957, "yaw": -1.5839587450027466, "level_name": "L1", "index": 0}, "path": []}]}\n'

    async def test_parse_and_get_values(self):
        parsed_values = await fleet_state_parser(self.data)
        self.assertEqual(parsed_values["name"], "tinyRobot")
        self.assertEqual(
            parsed_values["payload"],
            '{"name": "tinyRobot", "robots": [{"name": "tinyRobot1", "model": "", "task_id": "", "seq": 3190, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1598, "nanosec": 184999999}, "x": 11.553672790527344, "y": -11.317496299743652, "yaw": -1.599777340888977, "level_name": "L1", "index": 0}, "path": []}, {"name": "tinyRobot2", "model": "", "task_id": "", "seq": 3191, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1598, "nanosec": 685999999}, "x": 15.157517433166504, "y": -11.228611946105957, "yaw": -1.5839587450027466, "level_name": "L1", "index": 0}, "path": []}]}\n',
        )
