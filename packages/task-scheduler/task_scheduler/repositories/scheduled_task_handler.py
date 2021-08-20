from typing import List

from task_scheduler.models.tortoise_models import ScheduledTask


class ScheduledTaskRepository:
    @staticmethod
    async def get() -> List[ScheduledTask]:
        """
        Get the list of scheduled tasks.

        :return: list of scheduled tasks
        """
        return ScheduledTask.all()

    @staticmethod
    async def delete(id):
        """
        Delete a scheduled task.

        :return: None
        """
        ScheduledTask.delete(id)

    @staticmethod
    async def create():
        """
        Create a scheduled task.

        :return: scheduled task
        """
        pass
