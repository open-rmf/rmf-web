from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary
from rmf_task_msgs.msg import TaskType as RmfTaskType
from rmf_task_msgs.srv import CancelTask, GetTaskList, SubmitTask

from ... import models as mdl
from ..test_fixtures import RouteFixture
from .dispatcher import DispatcherClient

dispatcher_client = DispatcherClient()


class TestDispatcherClient(RouteFixture):
    def test_convert_task_request(self):
        task = mdl.SubmitTask(
            task_type=RmfTaskType.TYPE_CLEAN,
            start_time=0,
            description=mdl.CleanTaskDescription(cleaning_zone="zone_2"),
        )
        result, err = dispatcher_client.convert_task_request(task)
        self.assertEqual(err, "")
        self.assertIsNotNone(result)

        task = mdl.SubmitTask(
            task_type=RmfTaskType.TYPE_LOOP,
            start_time=0,
            description=mdl.LoopTaskDescription(
                num_loops=1, start_name="start", finish_name="finish"
            ),
        )
        result, err = dispatcher_client.convert_task_request(task)
        self.assertEqual(err, "")
        self.assertIsNotNone(result)

        task = mdl.SubmitTask(
            task_type=RmfTaskType.TYPE_DELIVERY,
            start_time=0,
            description=mdl.DeliveryTaskDescription(
                pickup_place_name="coe",
                pickup_dispenser="coke_dispenser",
                dropoff_ingestor="coke_ingestor",
                dropoff_place_name="supplies",
            ),
        )
        result, err = dispatcher_client.convert_task_request(task)
        self.assertEqual(err, "")
        self.assertIsNotNone(result)

    async def test_submit_task_request(self):
        # create a submit task request message
        task = mdl.SubmitTask(
            task_type=RmfTaskType.TYPE_CLEAN,
            start_time=0,
            description=mdl.CleanTaskDescription(cleaning_zone="zone_2"),
            priority=0,
        )
        req_msg, _err_msg = dispatcher_client.convert_task_request(task)
        fut = self.host_service_one(SubmitTask, "submit_task", SubmitTask.Response())
        await dispatcher_client.submit_task_request(req_msg)
        # checks that a service request is received
        await fut

    async def test_cancel_task_request(self):
        fut = self.host_service_one(CancelTask, "cancel_task", CancelTask.Response())
        await dispatcher_client.cancel_task_request(mdl.CancelTask(task_id="test_task"))
        # checks that a service request is received
        received: CancelTask.Request = await fut
        self.assertEqual(received.task_id, "test_task")

    async def test_get_task_status(self):
        self.host_service_one(
            GetTaskList,
            "get_tasks",
            GetTaskList.Response(active_tasks=[RmfTaskSummary(task_id="test_task")]),
        )
        result = await dispatcher_client.get_task_status()
        self.assertTrue(isinstance(result, list))
