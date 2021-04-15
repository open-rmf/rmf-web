import unittest

from models.door_state import DoorState

from .doors_state_parser import doors_state_parser


class TestCaseDoorsState(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.data = 'door_state: {"door_time": {"sec": 1596, "nanosec": 548000000}, "door_name": "hardware_door", "current_mode": {"value": 0}}\n'

    async def test_parse_to_json(self):
        state = await doors_state_parser(self.data)
        self.assertEqual(state["state"], DoorState.service.get_state_name(0))
