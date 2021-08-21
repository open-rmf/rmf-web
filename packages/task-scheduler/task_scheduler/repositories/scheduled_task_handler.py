from typing import List

from task_scheduler.models.pydantic_models import ScheduledTask_Pydantic
from task_scheduler.models.tortoise_models import ScheduledTask


class ScheduledTaskRepository:
    @staticmethod
    async def get(
        offset: int, limit: int, related_rule_id: int = None
    ) -> List[ScheduledTask]:
        """
        Get the list of scheduled tasks.

        :return: list of scheduled tasks
        """
        query = {}

        if related_rule_id:
            query["rule__id"] = related_rule_id

        queryset = (
            ScheduledTask.filter(**query)
            .prefetch_related("rule")
            .offset(offset)
            .limit(limit)
            .order_by("-created_at")
        )

        return await ScheduledTask_Pydantic.from_queryset(queryset)

    @staticmethod
    async def delete(id):
        """
        Delete a scheduled task.

        :return: None
        """
        await ScheduledTask.filter(id=id).delete()

    @staticmethod
    async def create():
        """
        Create a scheduled task.

        :return: scheduled task
        """
        pass
