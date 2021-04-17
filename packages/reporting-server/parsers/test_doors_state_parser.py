import unittest

from models.door_state import DoorState

from .doors_state_parser import doors_state_parser


class TestCaseDoorsState(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.data = 'door_state:{"door_time": {"sec": 1596, "nanosec": 548000000}, "door_name": "hardware_door", "current_mode": {"value": 0}}\n'

    async def test_parse_and_get_values(self):
        parsed_values = await doors_state_parser(self.data)
        self.assertEqual(parsed_values["state"], DoorState.service.get_state_name(0))
        self.assertEqual(parsed_values["name"], "hardware_door")
        self.assertEqual(
            parsed_values["payload"],
            '{"door_time": {"sec": 1596, "nanosec": 548000000}, "door_name": "hardware_door", "current_mode": {"value": 0}}\n',
        )
