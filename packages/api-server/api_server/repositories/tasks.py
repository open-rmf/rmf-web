import uuid
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple, cast

from fastapi import Depends, HTTPException
from pydantic import BaseModel
from tortoise.exceptions import FieldError, IntegrityError
from tortoise.query_utils import Prefetch
from tortoise.queryset import QuerySet
from tortoise.transactions import in_transaction

from api_server.authenticator import user_dep
from api_server.logger import format_exception, logger
from api_server.models import (
    LogEntry,
    Pagination,
    Phases,
    TaskEventLog,
    TaskState,
    User,
)
from api_server.models import tortoise_models as ttm
from api_server.models.tortoise_models import TaskState as DbTaskState
from api_server.query import add_pagination


class TaskFavoriteOut(BaseModel):
    id: str
    name: str
    unix_millis_earliest_start_time: datetime
    priority: dict
    category: str
    description: dict
    user: str


class TaskRepository:
    def __init__(self, user: User):
        self.user = user

    async def save_task_state(self, task_state: TaskState) -> None:
        await ttm.TaskState.update_or_create(
            {
                "data": task_state.json(),
                "category": task_state.category.__root__
                if task_state.category
                else None,
                "assigned_to": task_state.assigned_to.name
                if task_state.assigned_to
                else None,
                "unix_millis_start_time": task_state.unix_millis_start_time
                and datetime.fromtimestamp(task_state.unix_millis_start_time / 1000),
                "unix_millis_finish_time": task_state.unix_millis_finish_time
                and datetime.fromtimestamp(task_state.unix_millis_finish_time / 1000),
                "status": task_state.status if task_state.status else None,
            },
            id_=task_state.booking.id,
        )

    async def query_task_states(
        self, query: QuerySet[DbTaskState], pagination: Optional[Pagination] = None
    ) -> List[TaskState]:
        try:
            if pagination:
                query = add_pagination(query, pagination)
            # TODO: enforce with authz
            results = await query.values_list("data", flat=True)
            return [TaskState(**r) for r in results]
        except FieldError as e:
            raise HTTPException(422, str(e)) from e

    async def get_task_state(self, task_id: str) -> Optional[TaskState]:
        # TODO: enforce with authz
        result = await DbTaskState.get_or_none(id_=task_id)
        if result is None:
            return None
        return TaskState(**result.data)

    async def get_task_log(
        self, task_id: str, between: Tuple[int, int]
    ) -> Optional[TaskEventLog]:
        """
        :param between: The period in unix millis to fetch.
        """
        between_filters = {
            "unix_millis_time__gte": between[0],
            "unix_millis_time__lte": between[1],
        }
        result = cast(
            Optional[ttm.TaskEventLog],
            await ttm.TaskEventLog.get_or_none(task_id=task_id).prefetch_related(
                Prefetch(
                    "log",
                    ttm.TaskEventLogLog.filter(**between_filters),
                ),
                Prefetch(
                    "phases__log", ttm.TaskEventLogPhasesLog.filter(**between_filters)
                ),
                Prefetch(
                    "phases__events__log",
                    ttm.TaskEventLogPhasesEventsLog.filter(**between_filters),
                ),
            ),
        )
        if result is None:
            return None
        phases = {}
        for db_phase in result.phases:
            phase = {}
            phase["log"] = [LogEntry(**dict(x)) for x in db_phase.log]
            events = {}
            for db_event in db_phase.events:
                events[db_event.event] = [LogEntry(**dict(x)) for x in db_event.log]
            phase["events"] = events
            phases[db_phase.phase] = phase
        return TaskEventLog.construct(
            task_id=result.task_id,
            log=[LogEntry(**dict(x)) for x in result.log],
            phases=phases,
        )

    async def _saveEventLogs(
        self,
        db_phase: ttm.TaskEventLogPhases,
        events: Dict[str, List[LogEntry]],
    ):
        for event_id, logs in events.items():
            db_event = (
                await ttm.TaskEventLogPhasesEvents.get_or_create(
                    phase=db_phase, event=event_id
                )
            )[0]
            for log in logs:
                await ttm.TaskEventLogPhasesEventsLog.create(
                    event=db_event, **log.dict()
                )

    async def _savePhaseLogs(
        self, db_task_log: ttm.TaskEventLog, phases: Dict[str, Phases]
    ):
        for phase_id, phase in phases.items():
            db_phase = (
                await ttm.TaskEventLogPhases.get_or_create(
                    task=db_task_log, phase=phase_id
                )
            )[0]
            if phase.log:
                for log in phase.log:
                    await ttm.TaskEventLogPhasesLog.create(
                        phase=db_phase,
                        **log.dict(),
                    )
            if phase.events:
                await self._saveEventLogs(db_phase, phase.events)

    async def _saveTaskLogs(
        self, db_task_log: ttm.TaskEventLog, logs: Sequence[LogEntry]
    ):
        for log in logs:
            await ttm.TaskEventLogLog.create(
                task=db_task_log,
                seq=log.seq,
                unix_millis_time=log.unix_millis_time,
                tier=log.tier.name,
                text=log.text,
            )

    async def save_task_log(self, task_log: TaskEventLog) -> None:
        async with in_transaction():
            db_task_log = (
                await ttm.TaskEventLog.get_or_create(task_id=task_log.task_id)
            )[0]
            try:
                if task_log.log:
                    await self._saveTaskLogs(db_task_log, task_log.log)
                if task_log.phases:
                    await self._savePhaseLogs(db_task_log, task_log.phases)
            except IntegrityError as e:
                logger.error(format_exception(e))

    async def save_task_favorite(self, task_favorite: ttm.TaskFavoritePydantic) -> None:
        await ttm.TaskFavorite.update_or_create(
            {
                "name": task_favorite.name,
                "unix_millis_earliest_start_time": task_favorite.unix_millis_earliest_start_time,
                "priority": task_favorite.priority if task_favorite.priority else None,
                "category": task_favorite.category,
                "description": task_favorite.description
                if task_favorite.description
                else None,
                "user": self.user.username,
            },
            id=task_favorite.id if task_favorite.id != "" else uuid.uuid4(),
        )

    async def get_all_favorites_tasks(self) -> List[TaskFavoriteOut]:
        favorites_tasks = await ttm.TaskFavorite.filter(user=self.user.username)
        return [
            TaskFavoriteOut(
                id=favorite_task.id,
                name=favorite_task.name,
                unix_millis_earliest_start_time=int(
                    favorite_task.unix_millis_earliest_start_time.strftime(
                        "%Y%m%d%H%M%S"
                    )
                )
                if favorite_task.unix_millis_earliest_start_time
                else None,
                priority=favorite_task.priority if favorite_task.priority else None,
                category=favorite_task.category,
                description=favorite_task.description
                if favorite_task.description
                else None,
                user=self.user.username,
            )
            for favorite_task in favorites_tasks
        ]

    async def get_favorite_task_by_id(self, favorite_task_id: str) -> ttm.TaskFavorite:
        favorite_task = await ttm.TaskFavorite.get_or_none(id=favorite_task_id)
        if favorite_task is None:
            raise HTTPException(
                404, f"Favorite task with id {favorite_task_id} does not exists"
            )
        return favorite_task


def task_repo_dep(user: User = Depends(user_dep)):
    return TaskRepository(user)
