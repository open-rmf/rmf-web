import unittest

from rmf_task_msgs.msg import TaskSummary

from . import RmfGateway


class TestRmfGateway_TaskSummaries(unittest.TestCase):
    def setUp(self):
        self.rmf = RmfGateway()

    def test_keep_states(self):
        queued = TaskSummary(task_id="queued_task", state=TaskSummary.STATE_QUEUED)
        self.rmf.task_summaries.on_next(queued)
        active = TaskSummary(task_id="active_task", state=TaskSummary.STATE_ACTIVE)
        self.rmf.task_summaries.on_next(active)
        pending = TaskSummary(task_id="pending_task", state=TaskSummary.STATE_PENDING)
        self.rmf.task_summaries.on_next(pending)

        self.assertEqual(len(self.rmf.current_task_summaries), 3)

    def test_remove_completed_tasks(self):
        task = TaskSummary(task_id="test_task", state=TaskSummary.STATE_ACTIVE)
        self.rmf.task_summaries.on_next(task)
        self.assertEqual(len(self.rmf.current_task_summaries), 1)

        task = TaskSummary(task_id="test_task", state=TaskSummary.STATE_COMPLETED)
        self.rmf.task_summaries.on_next(task)
        self.assertEqual(len(self.rmf.current_task_summaries), 0)

    def test_remove_failed_tasks(self):
        task = TaskSummary(task_id="test_task", state=TaskSummary.STATE_ACTIVE)
        self.rmf.task_summaries.on_next(task)
        self.assertEqual(len(self.rmf.current_task_summaries), 1)

        task = TaskSummary(task_id="test_task", state=TaskSummary.STATE_FAILED)
        self.rmf.task_summaries.on_next(task)
        self.assertEqual(len(self.rmf.current_task_summaries), 0)

    def test_remove_cancelled_tasks(self):
        task = TaskSummary(task_id="test_task", state=TaskSummary.STATE_ACTIVE)
        self.rmf.task_summaries.on_next(task)
        self.assertEqual(len(self.rmf.current_task_summaries), 1)

        task = TaskSummary(task_id="test_task", state=TaskSummary.STATE_CANCELED)
        self.rmf.task_summaries.on_next(task)
        self.assertEqual(len(self.rmf.current_task_summaries), 0)
