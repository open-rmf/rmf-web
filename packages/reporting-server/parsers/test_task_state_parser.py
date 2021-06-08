import unittest

from models.task_state import TaskState

from .task_state_parser import task_state_parser


class TestCaseTaskState(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.data = 'task_state:{"tasks": [{"task_id": "1001", "fleet_name":"fleet_1", "submission_time": {"sec": 2000, "nanosec": 0}, "start_time": {"sec": 3000, "nanosec": 0}, "end_time": {"sec": 4000, "nanosec": 0}, "robot_name": "tinyRobot2", "task_type": 1, "state": 1, "priority": 0}]}\n'

    async def test_parse_and_get_values(self):
        parsed_values = await task_state_parser(self.data)

        self.assertEqual(len(parsed_values), 1)
        first_task = parsed_values[0]
        print(first_task["state"])

        self.assertEqual(first_task["task_id"], "1001")
        self.assertEqual(first_task["fleet_name"], "fleet_1")
        self.assertEqual(first_task["robot_name"], "tinyRobot2")
        self.assertEqual(first_task["submission_time"], {"sec": 2000, "nanosec": 0})
        self.assertEqual(first_task["state"], TaskState.service.get_task_state_name(1))
        self.assertEqual(
            first_task["task_type"], TaskState.service.get_task_type_name(1)
        )
        self.assertEqual(
            first_task["payload"],
            '{"tasks": [{"task_id": "1001", "fleet_name":"fleet_1", "submission_time": {"sec": 2000, "nanosec": 0}, "start_time": {"sec": 3000, "nanosec": 0}, "end_time": {"sec": 4000, "nanosec": 0}, "robot_name": "tinyRobot2", "task_type": 1, "state": 1, "priority": 0}]}\n',
        )

    async def test_return_empty_list_if_no_task(self):
        parsed_values = await task_state_parser('task_state:{"tasks":[]}')
        self.assertEqual(parsed_values, [])
