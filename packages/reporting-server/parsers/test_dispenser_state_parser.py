import unittest

from models.dispenser_state import DispenserState
from rest_server.__mocks__.parsed_data import mock_dispenser_state

from .dispenser_state_parser import dispenser_state_parser


class TestCaseDispenserState(unittest.IsolatedAsyncioTestCase):
    def setUp(self):

        self.data = mock_dispenser_state

    async def test_parse_and_get_values(self):
        parsed_values = await dispenser_state_parser(self.data)
        self.assertEqual(
            parsed_values["state"], DispenserState.service.get_state_name(0)
        )
        self.assertEqual(
            parsed_values["payload"],
            '{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_dispenser", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n',
        )
        self.assertEqual(parsed_values["guid"], "coke_dispenser")
