import unittest

from models.ingestor_state import IngestorState

from .ingestor_state_parser import ingestor_state_parser

# {'log': 'INFO:app.BookKeeper.ingestor_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_ingestor", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n', 'stream': 'stdout'}


class TestCaseIngestorState(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.data = 'ingestor_state:{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_ingestor", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n'

    async def test_parse_and_get_values(self):
        parsed_values = await ingestor_state_parser(self.data)
        self.assertEqual(parsed_values["guid"], "coke_ingestor")
        self.assertEqual(
            parsed_values["state"], IngestorState.service.get_state_name(0)
        )
        self.assertEqual(
            parsed_values["payload"],
            '{"time": {"sec": 1600, "nanosec": 0}, "guid": "coke_ingestor", "mode": 0, "request_guid_queue": [], "seconds_remaining": 0.0}\n',
        )
