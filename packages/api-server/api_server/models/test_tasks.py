import unittest

from pydantic import ValidationError

from .tasks import SubmitTask


class TestSubmitTask(unittest.TestCase):
    @staticmethod
    def validate_task(task_type: str, desc):
        SubmitTask.validate(
            {
                "task_type": task_type,
                "start_time": 0,
                "description": desc,
            }
        )

    def test_validate_task_description(self):
        clean_desc = {"cleaning_zone": "test_zone"}
        loop_desc = {
            "num_loops": 1,
            "start_name": "start",
            "finish_name": "finish",
        }
        delivery_desc = {
            "pickup_place_name": "pickup_place",
            "pickup_dispenser": "pickup_dispenser",
            "dropoff_ingestor": "dropoff_ingestor",
            "dropoff_place_name": "dropoff_place_name",
        }

        task_types = {
            "clean": clean_desc,
            "loop": loop_desc,
            "delivery": delivery_desc,
        }

        for task_type in task_types:
            for desc in task_types.values():
                # success when task type matches description
                if task_types[task_type] is desc:
                    TestSubmitTask.validate_task(task_type, desc)
                else:
                    # fails when sending description with wrong task type
                    self.assertRaises(
                        ValidationError, TestSubmitTask.validate_task, task_type, desc
                    )
