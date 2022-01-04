from typing import List, Optional

from api_server.authenticator import user_dep
from api_server.models import Pagination, TaskEventLog, TaskState, User
from api_server.models.tortoise_models import TaskEventLog as DbTaskEventLog
from api_server.models.tortoise_models import TaskState as DbTaskState
from api_server.query import add_pagination
from fastapi import Depends
from tortoise.queryset import QuerySet


class TaskRepository:
    def __init__(self, user: User):
        self.user = user

    async def query_task_states(
        self, query: QuerySet[DbTaskState], pagination: Optional[Pagination] = None
    ) -> List[TaskState]:
        if pagination:
            query = add_pagination(query, pagination)
        # TODO: enforce with authz
        results = await query.values_list("data", flat=True)
        return [TaskState(**r) for r in results]

    async def get_task_state(self, task_id: str) -> Optional[TaskState]:
        # TODO: enforce with authz
        result = await DbTaskState.get_or_none(id_=task_id)
        if result is None:
            return None
        return TaskState(**result.data)

    async def get_task_log(self, task_id: str) -> Optional[TaskEventLog]:
        result = await DbTaskEventLog.get_or_none(task_id=task_id)
        if result is None:
            return None
        return TaskEventLog(**result.data)


def task_repo_dep(user: User = Depends(user_dep)):
    return TaskRepository(user)
