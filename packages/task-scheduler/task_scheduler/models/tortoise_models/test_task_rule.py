# conflicts with isort because of local non-relative import
# pylint: disable=wrong-import-order
import datetime
import unittest

from fastapi.testclient import TestClient
from tortoise import Tortoise

from task_scheduler.app import get_app
from task_scheduler.test_utils import start_test_database

from .task_rule import FrequencyEnum, TaskRule, TaskTypeEnum

app = get_app()


class TestCaseTaskRuleEffect(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await start_test_database()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_create_task_rule_correctly(self):
        task_rule = await TaskRule.create(
            description="test",
            task_type=TaskTypeEnum.LOOP,
            frequency=1,
            frequency_type=FrequencyEnum.ONCE,
            time_of_day=datetime.datetime.now(),
            start_date=datetime.datetime.now(),
            # args=[],
        )

        self.assertEqual(task_rule.description, "test")

    async def test_create_task_rule_generates_scheduled_task(self):
        pass

    async def test_delete_task_rule_deletes_scheduled_task(self):
        pass

    async def test_raw_log_handle_creation_of_logs_with_container_name(self):
        pass
        # await RawLogHandler.create_raw_log([raw_data.mock_dispenser_state])
        # log = await RawLog.first().prefetch_related("container")
        # self.assertEqual(log.container.name, "app-that-writes-logs")


class TestCaseTaskRuleTaskGeneration(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await start_test_database()
        self.client = TestClient(app)

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_create_task_once(self):
        pass

    async def test_create_task_minutely(self):
        pass

    async def test_create_task_hourly(self):
        pass
