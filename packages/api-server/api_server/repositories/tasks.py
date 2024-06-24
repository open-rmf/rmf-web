import sys
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

import tortoise.functions as tfuncs
from fastapi import Depends, HTTPException
from tortoise.exceptions import FieldError, IntegrityError
from tortoise.expressions import Expression, Q
from tortoise.query_utils import Prefetch
from tortoise.transactions import in_transaction

from api_server.authenticator import user_dep
from api_server.logger import format_exception, logger
from api_server.models import (
    Labels,
    LogEntry,
    Pagination,
    Phases,
    TaskEventLog,
    TaskRequest,
    TaskState,
    TaskStatus,
    User,
)
from api_server.models import tortoise_models as ttm
from api_server.models.rmf_api.log_entry import Tier
from api_server.models.rmf_api.task_state import Category, Id, Phase
from api_server.models.tortoise_models import TaskRequest as DbTaskRequest
from api_server.models.tortoise_models import TaskState as DbTaskState
from api_server.rmf_io import task_events


class TaskRepository:
    def __init__(self, user: User):
        self.user = user

    async def save_task_request(self, task_id: str, task_request: TaskRequest) -> None:
        await DbTaskRequest.update_or_create(
            {"request": task_request.model_dump_json()}, id_=task_id
        )

    async def get_task_request(self, task_id: str) -> Optional[TaskRequest]:
        result = await DbTaskRequest.get_or_none(id_=task_id)
        if result is None:
            return None
        if not isinstance(result.request, dict):
            logger.error(f"request is not a dict: {type(result.request)}")
            raise HTTPException(500)
        return TaskRequest(**result.request)

    async def save_task_labels(
        self, db_task_state: ttm.TaskState, labels: Labels
    ) -> None:
        for k, v in labels.root.items():
            await ttm.TaskLabel.update_or_create(
                {"label_value": v},
                state=db_task_state,
                label_name=k,
            )

    async def save_task_state(self, task_state: TaskState) -> None:
        async with in_transaction():
            db_task_state, created = await DbTaskState.update_or_create(
                {
                    "data": task_state.model_dump_json(),
                    "category": task_state.category.root
                    if task_state.category
                    else None,
                    "assigned_to": task_state.assigned_to.name
                    if task_state.assigned_to
                    else None,
                    "unix_millis_start_time": task_state.unix_millis_start_time
                    and datetime.fromtimestamp(
                        task_state.unix_millis_start_time / 1000
                    ),
                    "unix_millis_finish_time": task_state.unix_millis_finish_time
                    and datetime.fromtimestamp(
                        task_state.unix_millis_finish_time / 1000
                    ),
                    "status": task_state.status if task_state.status else None,
                    "unix_millis_request_time": task_state.booking.unix_millis_request_time
                    and datetime.fromtimestamp(
                        task_state.booking.unix_millis_request_time / 1000
                    ),
                    "requester": task_state.booking.requester
                    if task_state.booking.requester
                    else None,
                },
                id_=task_state.booking.id,
            )

            # Labels attached to a task is not expected to change in task state updates,
            # so we can skip saving labels if it is not new. Note that if the labels were
            # to change, the labels we stored for querying would become out of sync.
            if created and task_state.booking.labels:
                labels = Labels.from_strings(task_state.booking.labels)
                await self.save_task_labels(db_task_state, labels)

    async def query_task_states(
        self,
        task_id: list[str] | None = None,
        category: list[str] | None = None,
        assigned_to: list[str] | None = None,
        start_time_between: tuple[datetime, datetime] | None = None,
        finish_time_between: tuple[datetime, datetime] | None = None,
        status: list[str] | None = None,
        label: Labels | None = None,
        pagination: Optional[Pagination] = None,
    ) -> List[TaskState]:
        filters = {}
        if task_id is not None:
            filters["id___in"] = task_id
        if category is not None:
            filters["category__in"] = category
        if assigned_to is not None:
            filters["assigned_to__in"] = assigned_to
        if start_time_between is not None:
            filters["unix_millis_start_time__gte"] = start_time_between[0]
            filters["unix_millis_start_time__lte"] = start_time_between[1]
        if finish_time_between is not None:
            filters["unix_millis_finish_time__gte"] = finish_time_between[0]
            filters["unix_millis_finish_time__lte"] = finish_time_between[1]
        if status is not None:
            valid_values = [member.value for member in TaskStatus]
            filters["status__in"] = []
            for status_string in status:
                if status_string not in valid_values:
                    continue
                filters["status__in"].append(TaskStatus(status_string))
        query = DbTaskState.filter(**filters)

        need_group_by = False
        label_filters = {}
        if label is not None:
            label_filters.update(
                {
                    f"label_filter_{k}": tfuncs.Count(
                        "id_",
                        _filter=Q(labels__label_name=k, labels__label_value=v),
                    )
                    for k, v in label.root.items()
                }
            )

        if len(label_filters) > 0:
            filter_gt = {f"{f}__gt": 0 for f in label_filters}
            query = query.annotate(**label_filters).filter(**filter_gt)
            need_group_by = True

        if pagination:
            order_fields: list[str] = []
            annotations: dict[str, Expression] = {}
            # add annotations required for sorting by labels
            for f in pagination.order_by:
                order_prefix = f[0] if f[0] == "-" else ""
                order_field = f[1:] if order_prefix == "-" else f
                if order_field.startswith("label="):
                    f = order_field[6:]
                    annotations[f"label_sort_{f}"] = tfuncs.Max(
                        "labels__label_value",
                        _filter=Q(labels__label_name=f),
                    )
                    order_field = f"label_sort_{f}"

                order_fields.append(order_prefix + order_field)

            query = (
                query.annotate(**annotations)
                .limit(pagination.limit)
                .offset(pagination.offset)
                .order_by(*order_fields)
            )
            need_group_by = True

        if need_group_by:
            query = query.group_by("id_", "labels__state_id")

        try:
            # TODO: enforce with authz
            results = await query.values_list("data")
            return [TaskState(**r[0]) for r in results]
        except FieldError as e:
            raise HTTPException(422, str(e)) from e

    async def get_task_state(self, task_id: str) -> Optional[TaskState]:
        # TODO: enforce with authz
        result = await DbTaskState.get_or_none(id_=task_id)
        if result is None:
            return None
        if not isinstance(result.data, dict):
            logger.error(f"data is not a dict: {type(result.data)}")
            raise HTTPException(500)
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
        result = await ttm.TaskEventLog.get_or_none(task_id=task_id).prefetch_related(
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
        )
        if result is None:
            return None
        phases = {}
        for db_phase in result.phases:
            phase = Phases(log=None, events=None)
            phase.log = [LogEntry(**dict(x)) for x in db_phase.log]
            events = {}
            for db_event in db_phase.events:
                events[db_event.event] = [LogEntry(**dict(x)) for x in db_event.log]
            phase.events = events
            phases[db_phase.phase] = phase
        return TaskEventLog.model_construct(
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
                    event=db_event, **log.model_dump()
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
                        **log.model_dump(),
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

    async def save_log_acknowledged_task_completion(
        self, task_id: str, acknowledged_by: str, unix_millis_acknowledged_time: int
    ) -> None:
        async with in_transaction():
            task_logs = await self.get_task_log(task_id, (0, sys.maxsize))
            task_state = await self.get_task_state(task_id=task_id)
            # A try could be used here to avoid using so many "ands"
            # but the configured lint suggests comparing that no value is None
            if task_logs and task_state and task_logs.phases and task_state.phases:
                # The next phase key value matches in both `task_logs` and `task_state`.
                #   It is the same whether it is obtained from `task_logs` or from `task_state`.
                #   In this case, it is obtained from `task_logs` and is also used to assign the next
                #   phase in `task_state`.
                next_phase_key = str(int(list(task_logs.phases)[-1]) + 1)
            else:
                raise ValueError("Phases can't be null")

            event = LogEntry(
                seq=0,
                tier=Tier.warning,
                unix_millis_time=unix_millis_acknowledged_time,
                text=f"Task completion acknowledged by {acknowledged_by}",
            )
            task_logs.phases = {
                **task_logs.phases,
                next_phase_key: Phases(log=[], events={"0": [event]}),
            }

            await self.save_task_log(task_logs)

            task_state.phases = {
                **task_state.phases,
                next_phase_key: Phase(
                    id=Id(root=next_phase_key),
                    category=Category(root="Task completed"),
                    detail=None,
                    unix_millis_start_time=None,
                    unix_millis_finish_time=None,
                    original_estimate_millis=None,
                    estimate_millis=None,
                    final_event_id=None,
                    events=None,
                    skip_requests=None,
                ),
            }

            await self.save_task_state(task_state)
            # Notifies observers of the next task_state value to correctly display the title of the
            #  logs when acknowledged by a user without reloading the page.
            task_events.task_states.on_next(task_state)

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


def task_repo_dep(user: User = Depends(user_dep)):
    return TaskRepository(user)
