import unittest

from models.task_state import TaskState

from .task_state_parser import task_state_parser


class TestCaseFleetState(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.data = 'task_state:{"tasks": [{"task_id": "1001", "task_type":"", "task_state",}, {"name": "tinyRobot2", "model": "", "task_id": "", "seq": 3191, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1598, "nanosec": 685999999}, "x": 15.157517433166504, "y": -11.228611946105957, "yaw": -1.5839587450027466, "level_name": "L1", "index": 0}, "path": []}]}\n'

    async def test_parse_and_get_values(self):
        parsed_values = await fleet_state_parser(self.data)

        self.assertEqual(len(parsed_values), 2)
        first_robot = parsed_values[0]
        second_robot = parsed_values[1]

        self.assertEqual(first_robot["fleet_name"], "tinyRobot")
        self.assertEqual(first_robot["robot_name"], "tinyRobot1")
        self.assertEqual(first_robot["robot_model"], "")
        self.assertEqual(first_robot["robot_task_id"], "")
        self.assertEqual(first_robot["robot_battery_percent"], 100.0)
        self.assertEqual(first_robot["robot_seq"], 3190)
        self.assertEqual(
            first_robot["robot_mode"], FleetState.service.get_robot_state_name(1)
        )
        self.assertEqual(
            first_robot["payload"],
            '{"name": "tinyRobot", "robots": [{"name": "tinyRobot1", "model": "", "task_id": "", "seq": 3190, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1598, "nanosec": 184999999}, "x": 11.553672790527344, "y": -11.317496299743652, "yaw": -1.599777340888977, "level_name": "L1", "index": 0}, "path": []}, {"name": "tinyRobot2", "model": "", "task_id": "", "seq": 3191, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1598, "nanosec": 685999999}, "x": 15.157517433166504, "y": -11.228611946105957, "yaw": -1.5839587450027466, "level_name": "L1", "index": 0}, "path": []}]}\n',
        )

        self.assertEqual(second_robot["fleet_name"], "tinyRobot")
        self.assertEqual(second_robot["robot_name"], "tinyRobot2")
        self.assertEqual(second_robot["robot_model"], "")
        self.assertEqual(second_robot["robot_task_id"], "")
        self.assertEqual(second_robot["robot_battery_percent"], 100.0)
        self.assertEqual(second_robot["robot_seq"], 3191)
        self.assertEqual(
            second_robot["robot_mode"], FleetState.service.get_robot_state_name(1)
        )

    async def test_return_empty_list_if_no_robot(self):
        parsed_values = await fleet_state_parser(
            'fleet_state:{"name": "tinyRobot", "robots":[]}'
        )
        self.assertEqual(parsed_values, [])
