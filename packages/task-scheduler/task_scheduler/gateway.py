import asyncio
import base64
import hashlib
import logging
import threading
from typing import List, Optional, cast

import rclpy.client
import rclpy.node
import rclpy.qos
from fastapi import HTTPException
from rclpy.subscription import Subscription
from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary
from rmf_task_msgs.srv import CancelTask as RmfCancelTask
from rmf_task_msgs.srv import GetTaskList as RmfGetTaskList
from rmf_task_msgs.srv import SubmitTask as RmfSubmitTask
from rosidl_runtime_py.convert import message_to_ordereddict

from .models import TaskSummary
from .repositories import RmfRepository, StaticFilesRepository
from .rmf_io import RmfEvents


class RmfGateway(rclpy.node.Node):
    def __init__(
        self,
        rmf_events: RmfEvents,
        *,
        logger: logging.Logger = None,
    ):
        super().__init__("rmf_task_scheduler")

        self.submit_task_srv = self.create_client(RmfSubmitTask, "submit_task")
        self.get_tasks_srv = self.create_client(RmfGetTaskList, "get_tasks")
        self.cancel_task_srv = self.create_client(RmfCancelTask, "cancel_task")

        self.rmf_events = rmf_events
        self.logger = logger or logging.getLogger(self.__class__.__name__)
        self._subscriptions = cast(List[Subscription], [])
        self._spin_thread: Optional[threading.Thread] = None
        self._stopping = False
        self._loop: asyncio.AbstractEventLoop = None

    async def call_service(self, client: rclpy.client.Client, req, timeout=1):
        """
        Utility to wrap a ros service call in an awaitable,
        raises HTTPException if service call fails.
        """
        fut = client.call_async(req)
        try:
            result = await asyncio.wait_for(fut, timeout=timeout)
            return result
        except asyncio.TimeoutError as e:
            raise HTTPException(503, "ros service call timed out") from e

    def subscribe_all(self):

        task_summaries_sub = self.create_subscription(
            RmfTaskSummary,
            "task_summaries",
            lambda msg: self.rmf_events.task_summaries.on_next(
                TaskSummary.from_orm(msg)
            ),
            10,
        )
        self._subscriptions.append(task_summaries_sub)

    def unsubscribe_all(self):
        for sub in self._subscriptions:
            sub.destroy()
        self._subscriptions = []

    async def update_tasks(self, repo: RmfRepository):
        """
        Updates the tasks in a RmfRepository with the current tasks in RMF.
        """
        resp: RmfGetTaskList.Response = await self.call_service(
            self.get_tasks_srv, RmfGetTaskList.Request()
        )
        if not resp.success:
            raise HTTPException(500, "service call succeeded but RMF returned an error")
        for task_summary in resp.active_tasks:
            task_summary: RmfTaskSummary
            await repo.save_task_summary(task_summary)
        for task_summary in resp.terminated_tasks:
            task_summary: RmfTaskSummary
            await repo.save_task_summary(task_summary)
