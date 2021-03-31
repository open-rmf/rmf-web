import unittest
from unittest.mock import MagicMock

import rclpy
from rclpy.node import Client, Node

from .dispatcher import DispatcherClient

Client.wait_for_service = MagicMock(return_value=True)
rclpy.init(args=None)
dispatcher_client = DispatcherClient()


class TestDispatcherClient(unittest.TestCase):
    def test_convert_task_request(self):
        mock_task_json_1 = {"task_type": "Loop", "start_time": 0}
        req_msg_1, err_msg_1 = dispatcher_client.convert_task_request(mock_task_json_1)
        self.assertEqual(err_msg_1, "Key value is incomplete")
        self.assertEqual(req_msg_1, None)

        mock_task_json_2 = {"task_type": "Loop", "description": {}}
        req_msg_2, err_msg_2 = dispatcher_client.convert_task_request(mock_task_json_2)
        self.assertEqual(err_msg_2, "Key value is incomplete")
        self.assertEqual(req_msg_2, None)

        mock_task_json_3 = {"start_time": 0, "description": {}}
        req_msg_3, err_msg_3 = dispatcher_client.convert_task_request(mock_task_json_3)
        self.assertEqual(err_msg_3, "Key value is incomplete")
        self.assertEqual(req_msg_3, None)

        mock_task_json_4 = {
            "task_type": "Loop",
            "start_time": 0,
            "description": {},
            "priority": "-1",
        }
        req_msg_4, err_msg_4 = dispatcher_client.convert_task_request(mock_task_json_4)
        self.assertEqual(err_msg_4, "Priority value is less than 0")
        self.assertEqual(req_msg_4, None)

        mock_task_json_5 = {
            "task_type": "Clean",
            "start_time": 0,
            "description": {"cleaning_zone": "zone_2"},
            "priority": "0",
        }
        req_msg_5, err_msg_5 = dispatcher_client.convert_task_request(mock_task_json_5)
        self.assertEqual(err_msg_5, "")
        self.assertIsNotNone(req_msg_5)

        mock_task_json_6 = {
            "task_type": "Loop",
            "start_time": 0,
            "description": {
                "num_loops": 1,
                "start_name": "start",
                "finish_name": "finish",
            },
        }
        req_msg_6, err_msg_6 = dispatcher_client.convert_task_request(mock_task_json_6)
        self.assertEqual(err_msg_6, "")
        self.assertIsNotNone(req_msg_6)

        mock_task_json_7 = {
            "task_type": "Delivery",
            "start_time": 0,
            "description": {
                "option": "coke",
                "pickup_place_name": "coe",
                "pickup_dispenser": "coke_dispenser",
                "dropoff_ingestor": "coke_ingestor",
                "dropoff_place_name": "supplies",
            },
        }
        req_msg_7, err_msg_7 = dispatcher_client.convert_task_request(mock_task_json_7)
        self.assertEqual(err_msg_7, "")
        self.assertIsNotNone(req_msg_7)

        mock_task_json_8 = {
            "task_type": "wrong_task",
            "start_time": 0,
            "description": {},
        }
        req_msg_8, err_msg_8 = dispatcher_client.convert_task_request(mock_task_json_8)
        self.assertEqual(err_msg_8, "Invalid TaskType")
        self.assertEqual(req_msg_8, None)

        mock_task_json_9 = {
            "task_type": "Loop",
            "start_time": 0,
            "description": {"start_name": "start", "finish_name": "finish"},
        }
        req_msg_9, err_msg_9 = dispatcher_client.convert_task_request(mock_task_json_9)
        self.assertEqual(err_msg_9, "Missing Key value in json body: 'num_loops'")
        self.assertEqual(req_msg_9, None)

    def test_submit_task_request(self):
        class MockResult:
            success = True
            task_id = "test_task_id"

        class MockFuture:
            def done(self):
                return True

            def result(self):
                return MockResult()

        # create a submit task request message
        mock_task_json = {
            "task_type": "Clean",
            "start_time": 0,
            "description": {"cleaning_zone": "zone_2"},
            "priority": "0",
        }
        Client.call_async = MagicMock(return_value=MockFuture())
        req_msg, err_msg = dispatcher_client.convert_task_request(mock_task_json)
        task_id = dispatcher_client.submit_task_request(req_msg)
        self.assertEqual(task_id, MockResult().task_id)

    def test_cancel_task_request(self):
        class MockResult:
            success = True
            task_id = "test_task_id"

        class MockFuture:
            def done(self):
                return True

            def result(self):
                return MockResult()

        Client.call_async = MagicMock(return_value=MockFuture())
        cancel_status = dispatcher_client.cancel_task_request("")
        self.assertEqual(cancel_status, MockResult().success)
