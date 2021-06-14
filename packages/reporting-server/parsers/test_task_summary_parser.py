import unittest

from models.task_summary import TaskSummary

from .task_summary_parser import task_summary_parser

# {'log': 'INFO:app.BookKeeper.task_summary:{"fleet_name": "tinyRobot", "task_id": "Loop0", "task_profile": {"task_id": "Loop0", "submission_time": {"sec": 131, "nanosec": 553000000}, "description": {"start_time": {"sec": 1623383402, "nanosec": 0}, "priority": {"value": 0}, "task_type": {"type": 1}, "station": {"task_id": "", "robot_type": "", "place_name": ""}, "loop": {"task_id": "", "robot_type": "", "num_loops": 1, "start_name": "supplies", "finish_name": "coe"}, "delivery": {"task_id": "", "items": [], "pickup_place_name": "", "pickup_dispenser": "", "pickup_behavior": {"name": "", "parameters": []}, "dropoff_place_name": "", "dropoff_ingestor": "", "dropoff_behavior": {"name": "", "parameters": []}}, "clean": {"start_waypoint": ""}}}, "state": 0, "status": "", "submission_time": {"sec": 0, "nanosec": 0}, "start_time": {"sec": 1623383362, "nanosec": 348338289}, "end_time": {"sec": 1623383449, "nanosec": 79154833}, "robot_name": "tinyRobot2"}


class TestCaseTaskSummary(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.data = 'task_summary:{"fleet_name": "tinyRobot", "task_id": "Loop0", "task_profile": {"task_id": "Loop0", "submission_time": {"sec": 131, "nanosec": 553000000}, "description": {"start_time": {"sec": 1623383402, "nanosec": 0}, "priority": {"value": 0}, "task_type": {"type": 1}, "station": {"task_id": "", "robot_type": "", "place_name": ""}, "loop": {"task_id": "", "robot_type": "", "num_loops": 1, "start_name": "supplies", "finish_name": "coe"}, "delivery": {"task_id": "", "items": [], "pickup_place_name": "", "pickup_dispenser": "", "pickup_behavior": {"name": "", "parameters": []}, "dropoff_place_name": "", "dropoff_ingestor": "", "dropoff_behavior": {"name": "", "parameters": []}}, "clean": {"start_waypoint": ""}}}, "state": 0, "status": "", "submission_time": {"sec": 0, "nanosec": 0}, "start_time": {"sec": 1623383362, "nanosec": 348338289}, "end_time": {"sec": 1623383449, "nanosec": 79154833}, "robot_name": "tinyRobot2"}\n'

    async def test_parse_and_get_values(self):
        parsed_values = await task_summary_parser(self.data)

        self.assertEqual(parsed_values["task_id"], "Loop0")
        self.assertEqual(parsed_values["fleet_name"], "tinyRobot")
        self.assertEqual(
            parsed_values["state"], TaskSummary.service.get_task_state_name(0)
        )
        self.assertEqual(
            parsed_values["task_profile"]["submission_time"],
            {"sec": 131, "nanosec": 553000000},
        )
        self.assertEqual(parsed_values["submission_time"], {"sec": 0, "nanosec": 0})
        self.assertEqual(
            parsed_values["start_time"], {"sec": 1623383362, "nanosec": 348338289}
        )
        self.assertEqual(
            parsed_values["end_time"], {"sec": 1623383449, "nanosec": 79154833}
        )
        self.assertEqual(parsed_values["robot_name"], "tinyRobot2")
        self.assertEqual(
            parsed_values["task_profile"]["description"]["task_type"]["type"], 1
        )
        self.assertEqual(
            parsed_values["payload"],
            '{"fleet_name": "tinyRobot", "task_id": "Loop0", "task_profile": {"task_id": "Loop0", "submission_time": {"sec": 131, "nanosec": 553000000}, "description": {"start_time": {"sec": 1623383402, "nanosec": 0}, "priority": {"value": 0}, "task_type": {"type": 1}, "station": {"task_id": "", "robot_type": "", "place_name": ""}, "loop": {"task_id": "", "robot_type": "", "num_loops": 1, "start_name": "supplies", "finish_name": "coe"}, "delivery": {"task_id": "", "items": [], "pickup_place_name": "", "pickup_dispenser": "", "pickup_behavior": {"name": "", "parameters": []}, "dropoff_place_name": "", "dropoff_ingestor": "", "dropoff_behavior": {"name": "", "parameters": []}}, "clean": {"start_waypoint": ""}}}, "state": 0, "status": "", "submission_time": {"sec": 0, "nanosec": 0}, "start_time": {"sec": 1623383362, "nanosec": 348338289}, "end_time": {"sec": 1623383449, "nanosec": 79154833}, "robot_name": "tinyRobot2"}\n',
        )

    async def test_return_empty_list_if_no_task(self):
        parsed_values = await task_summary_parser("task_summary:{}")
        self.assertEqual(parsed_values, {})
