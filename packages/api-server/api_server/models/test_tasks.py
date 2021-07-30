import unittest
from itertools import product

from pydantic import ValidationError
from rmf_task_msgs.msg import TaskType as RmfTaskType

from api_server.models.tasks import SubmitTask


class TestSubmitTaskModel(unittest.TestCase):
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
            RmfTaskType.TYPE_CLEAN: clean_desc,
            RmfTaskType.TYPE_LOOP: loop_desc,
            RmfTaskType.TYPE_DELIVERY: delivery_desc,
        }

        for task_type, desc in product(task_types, task_types.values()):
            # success when task type matches description
            if task_types[task_type] is desc:
                SubmitTask(
                    task_type=task_type,
                    start_time=0,
                    description=desc,
                )
            else:
                # fails when sending description with wrong task type
                self.assertRaises(
                    ValidationError,
                    SubmitTask,
                    task_type=task_type,
                    start_time=0,
                    desc=desc,
                )
