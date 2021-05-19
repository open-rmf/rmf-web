import unittest

from rmf_task_msgs.msg import TaskType as RmfTaskType

from ..models import (
    CleanTaskDescription,
    DeliveryTaskDescription,
    LoopTaskDescription,
    SubmitTask,
)
from .tasks import convert_task_request


class TestDispatcherClient(unittest.TestCase):
    def test_convert_task_request(self):
        task = SubmitTask(
            task_type=RmfTaskType.TYPE_CLEAN,
            start_time=0,
            description=CleanTaskDescription(cleaning_zone="zone_2"),
        )
        result, err = convert_task_request(task)
        self.assertEqual(err, "")
        self.assertIsNotNone(result)

        task = SubmitTask(
            task_type=RmfTaskType.TYPE_LOOP,
            start_time=0,
            description=LoopTaskDescription(
                num_loops=1, start_name="start", finish_name="finish"
            ),
        )
        result, err = convert_task_request(task)
        self.assertEqual(err, "")
        self.assertIsNotNone(result)

        task = SubmitTask(
            task_type=RmfTaskType.TYPE_DELIVERY,
            start_time=0,
            description=DeliveryTaskDescription(
                pickup_place_name="coe",
                pickup_dispenser="coke_dispenser",
                dropoff_ingestor="coke_ingestor",
                dropoff_place_name="supplies",
            ),
        )
        result, err = convert_task_request(task)
        self.assertEqual(err, "")
        self.assertIsNotNone(result)
