from typing import List

from tortoise.queryset import QuerySet

from api_server.models import TaskState
from api_server.models.tortoise_models import TaskState as DbTaskState
from api_server.models.user import User


class TaskRepository:
    def __init__(self, user: User):
        self.user = user

    async def get_task_states(self, query: QuerySet[DbTaskState]) -> List[TaskState]:
        # TODO: enforce with authz
        results = await query.values_list("data", flat=True)
        return [TaskState(**r) for r in results]
