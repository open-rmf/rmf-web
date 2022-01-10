from datetime import datetime
from typing import Any, Dict, Sequence

from tortoise.exceptions import IntegrityError
from tortoise.transactions import in_transaction

from api_server.logger import format_exception, logger

from . import tortoise_models as ttm
from .rmf_api.log_entry import LogEntry
from .rmf_api.task_log import TaskEventLog as BaseTaskEventLog
from .rmf_api.task_state import TaskState as BaseTaskState


class TaskState(BaseTaskState):
    @staticmethod
    def from_db(task_state: ttm.TaskState) -> "TaskState":
        return TaskState(**task_state.data)

    async def save(self) -> None:
        await ttm.TaskState.update_or_create(
            {
                "data": self.json(),
                "category": self.category,
                "unix_millis_start_time": self.unix_millis_start_time
                and datetime.fromtimestamp(self.unix_millis_start_time / 1000),
                "unix_millis_finish_time": self.unix_millis_finish_time
                and datetime.fromtimestamp(self.unix_millis_finish_time / 1000),
            },
            id_=self.booking["id"],
        )


class TaskEventLog(BaseTaskEventLog):
    async def _saveEventLogs(
        self,
        db_phase: ttm.TaskEventLogPhases,
        events: Dict[str, Sequence[Dict[str, Any]]],
    ):
        if events:
            for event_id, logs in events.items():
                db_event = (
                    await ttm.TaskEventLogPhasesEvents.get_or_create(
                        phase=db_phase, event=event_id
                    )
                )[0]
                for log in logs:
                    await ttm.TaskEventLogPhasesEventsLog.create(event=db_event, **log)

    async def _savePhaseLogs(
        self, db_task_log: ttm.TaskEventLog, phases: Dict[str, Dict[str, Dict]]
    ):
        for phase_id, phase in phases.items():
            db_phase = (
                await ttm.TaskEventLogPhases.get_or_create(
                    task=db_task_log, phase=phase_id
                )
            )[0]
            if "log" in phase:
                for log in phase["log"]:
                    await ttm.TaskEventLogPhasesLog.create(
                        phase=db_phase,
                        **log,
                    )
            if "events" in phase:
                await self._saveEventLogs(db_phase, phase["events"])

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

    async def save(self) -> None:
        async with in_transaction():
            db_task_log = (await ttm.TaskEventLog.get_or_create(task_id=self.task_id))[
                0
            ]
            try:
                if self.log:
                    await self._saveTaskLogs(db_task_log, self.log)
                if self.phases:
                    await self._savePhaseLogs(db_task_log, self.phases)
            except IntegrityError as e:
                logger.error(format_exception(e))