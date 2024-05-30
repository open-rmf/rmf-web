from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple, cast

from fastapi import Depends, HTTPException
from tortoise.exceptions import FieldError, IntegrityError
from tortoise.query_utils import Prefetch
from tortoise.queryset import QuerySet
from tortoise.transactions import in_transaction

from api_server.authenticator import user_dep
from api_server.logging import LoggerAdapter, get_logger
from api_server.models import (
    LogEntry,
    Pagination,
    Phases,
    TaskBookingLabel,
    TaskEventLog,
    TaskRequest,
    TaskState,
    User,
)
from api_server.models import tortoise_models as ttm
from api_server.models.tortoise_models import TaskRequest as DbTaskRequest
from api_server.models.tortoise_models import TaskState as DbTaskState
from api_server.query import add_pagination


class TaskRepository:
    def __init__(
        self,
        user: User = Depends(user_dep),
        logger: LoggerAdapter = Depends(get_logger),
    ):
        self.user = user
        self.logger = logger

    async def save_task_request(
        self, task_state: TaskState, task_request: TaskRequest
    ) -> None:
        await DbTaskRequest.update_or_create(
            {"request": task_request.json()}, id_=task_state.booking.id
        )

    async def get_task_request(self, task_id: str) -> Optional[TaskRequest]:
        result = await DbTaskRequest.get_or_none(id_=task_id)
        if result is None:
            return None
        return TaskRequest(**result.request)

    async def query_task_requests(self, task_ids: List[str]) -> List[DbTaskRequest]:
        filters = {"id___in": task_ids}
        try:
            return await DbTaskRequest.filter(**filters)
        except FieldError as e:
            raise HTTPException(422, str(e)) from e

    async def save_task_state(self, task_state: TaskState) -> None:
        task_state_dict = {
            "data": task_state.json(),
            "category": task_state.category.__root__ if task_state.category else None,
            "assigned_to": task_state.assigned_to.name
            if task_state.assigned_to
            else None,
            "unix_millis_start_time": task_state.unix_millis_start_time
            and datetime.fromtimestamp(task_state.unix_millis_start_time / 1000),
            "unix_millis_finish_time": task_state.unix_millis_finish_time
            and datetime.fromtimestamp(task_state.unix_millis_finish_time / 1000),
            "status": task_state.status if task_state.status else None,
            "unix_millis_request_time": task_state.booking.unix_millis_request_time
            and datetime.fromtimestamp(
                task_state.booking.unix_millis_request_time / 1000
            ),
            "requester": task_state.booking.requester
            if task_state.booking.requester
            else None,
        }

        try:
            state, created = await ttm.TaskState.update_or_create(
                task_state_dict, id_=task_state.booking.id
            )
        except Exception as e:  # pylint: disable=W0703
            # This is to catch a combination of exceptions from Tortoise ORM,
            # especially in the case where a data race occurs when two instances
            # of update_or_create attempts to create entries with the same task
            # ID at the same time. The exceptions expected are DoesNotExist,
            # IntegrityError and TransactionManagementError.
            # This data race happens when the server attempts to record the RMF
            # service call response and Task dispatcher's websocket push at
            # almost the same time.
            # FIXME: this has not been observed outside of production
            # environment, and may be fixed upstream in updated libraries.
            self.logger.error(
                f"Failed to save task state of id [{task_state.booking.id}] [{e}]"
            )
            return

        # Since this is updating an existing task state, we are done
        if not created:
            return

        # Labels are created and saved when a new task state is first received
        labels = task_state.booking.labels
        booking_label = None
        if labels is not None:
            for l in labels:
                validated_booking_label = TaskBookingLabel.from_json_string(l)
                if validated_booking_label is not None:
                    booking_label = validated_booking_label
                    break
        if booking_label is None:
            return

        # Here we generate the labels required for server-side sorting and
        # filtering.
        if booking_label.description.pickup is not None:
            await ttm.TaskLabel.create(
                state=state,
                label_name="pickup",
                label_value_str=booking_label.description.pickup,
            )
        if booking_label.description.destination is not None:
            await ttm.TaskLabel.create(
                state=state,
                label_name="destination",
                label_value_str=booking_label.description.destination,
            )
        if booking_label.description.unix_millis_warn_time is not None:
            await ttm.TaskLabel.create(
                state=state,
                label_name="unix_millis_warn_time",
                label_value_num=booking_label.description.unix_millis_warn_time,
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
            phase = Phases(log=None, events=None)
            phase.log = [LogEntry(**dict(x)) for x in db_phase.log]
            events = {}
            for db_event in db_phase.events:
                events[db_event.event] = [LogEntry(**dict(x)) for x in db_event.log]
            phase.events = events
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
                self.logger.error(e)
