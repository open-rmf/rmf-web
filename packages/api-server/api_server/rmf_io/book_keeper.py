import asyncio
import json
import logging
from collections import namedtuple
from typing import Coroutine, List, Optional

from rx.core.typing import Disposable
from rx.subject.subject import Subject

from api_server.models import (
    BuildingMap,
    DispenserHealth,
    DispenserState,
    DoorHealth,
    DoorState,
    HealthStatus,
    IngestorHealth,
    IngestorState,
    LiftHealth,
    LiftState,
)
from api_server.models.health import BaseBasicHealth

from .events import RmfEvents


class RmfBookKeeperEvents:
    def __init__(self):
        self.task_summary_written = Subject()  # TaskSummary


class RmfBookKeeper:
    _ChildLoggers = namedtuple(
        "_ChildLoggers",
        [
            "building_map",
            "door_state",
            "door_health",
            "lift_state",
            "lift_health",
            "dispenser_state",
            "dispenser_health",
            "ingestor_state",
            "ingestor_health",
            "fleet_state",
            "robot_health",
            "task_summary",
        ],
    )

    def __init__(
        self,
        rmf_events: RmfEvents,
        *,
        logger: Optional[logging.Logger] = None,
    ):
        self.rmf = rmf_events
        self.bookkeeper_events = RmfBookKeeperEvents()
        self._loop: asyncio.AbstractEventLoop
        self._main_logger = logger or logging.getLogger(self.__class__.__name__)
        self._pending_tasks = set()

        self._loggers = self._ChildLoggers(
            self._main_logger.getChild("building_map"),
            self._main_logger.getChild("door_state"),
            self._main_logger.getChild("door_health"),
            self._main_logger.getChild("lift_state"),
            self._main_logger.getChild("lift_health"),
            self._main_logger.getChild("dispenser_state"),
            self._main_logger.getChild("dispenser_health"),
            self._main_logger.getChild("ingestor_state"),
            self._main_logger.getChild("ingestor_health"),
            self._main_logger.getChild("fleet_state"),
            self._main_logger.getChild("robot_health"),
            self._main_logger.getChild("task_summary"),
        )

        self._loggers.door_state.parent = self._main_logger
        self._loggers.door_health.parent = self._main_logger
        self._loggers.lift_state.parent = self._main_logger
        self._loggers.lift_health.parent = self._main_logger
        self._loggers.dispenser_state.parent = self._main_logger
        self._loggers.dispenser_health.parent = self._main_logger
        self._loggers.ingestor_state.parent = self._main_logger
        self._loggers.ingestor_health.parent = self._main_logger
        self._loggers.fleet_state.parent = self._main_logger
        self._loggers.robot_health.parent = self._main_logger
        self._loggers.task_summary.parent = self._main_logger

        self._subscriptions: List[Disposable] = []

    async def start(self):
        self._loop = asyncio.get_event_loop()
        self._record_building_map()
        self._record_door_state()
        self._record_door_health()
        self._record_lift_state()
        self._record_lift_health()
        self._record_dispenser_state()
        self._record_dispenser_health()
        self._record_ingestor_state()
        self._record_ingestor_health()

    async def stop(self):
        for sub in self._subscriptions:
            sub.dispose()
        self._subscriptions.clear()
        if len(self._pending_tasks) > 0:
            await asyncio.wait(self._pending_tasks)

    def _create_task(self, coro: Coroutine):
        task = self._loop.create_task(coro)
        task.add_done_callback(self._pending_tasks.remove)
        self._pending_tasks.add(task)

    @staticmethod
    def _report_health(health: BaseBasicHealth, logger: logging.Logger):
        message = health.json()
        if health.health_status == HealthStatus.UNHEALTHY:
            logger.warning(message)
        elif health.health_status == HealthStatus.DEAD:
            logger.error(message)
        else:
            logger.info(message)

    def _record_building_map(self):
        async def update(building_map: BuildingMap):
            if not building_map:
                return
            await building_map.save()
            self._loggers.building_map.info(json.dumps(building_map.dict()))

        self._subscriptions.append(
            self.rmf.building_map.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_door_state(self):
        async def update(door_state: DoorState):
            await door_state.save()
            self._loggers.door_state.info(json.dumps(door_state.dict()))

        self._subscriptions.append(
            self.rmf.door_states.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_door_health(self):
        async def update(health: DoorHealth):
            await health.save()
            self._report_health(health, self._loggers.door_health)

        self._subscriptions.append(
            self.rmf.door_health.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_lift_state(self):
        async def update(lift_state: LiftState):
            await lift_state.save()
            self._loggers.lift_state.info(lift_state.json())

        self._subscriptions.append(
            self.rmf.lift_states.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_lift_health(self):
        async def update(health: LiftHealth):
            await health.save()
            self._report_health(health, self._loggers.lift_health)

        self._subscriptions.append(
            self.rmf.lift_health.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_dispenser_state(self):
        async def update(dispenser_state: DispenserState):
            await dispenser_state.save()
            self._loggers.dispenser_state.info(dispenser_state.json())

        self._subscriptions.append(
            self.rmf.dispenser_states.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_dispenser_health(self):
        async def update(health: DispenserHealth):
            await health.save()
            self._report_health(health, self._loggers.dispenser_health)

        self._subscriptions.append(
            self.rmf.dispenser_health.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_ingestor_state(self):
        async def update(ingestor_state: IngestorState):
            await ingestor_state.save()
            self._loggers.ingestor_state.info(ingestor_state.json())

        self._subscriptions.append(
            self.rmf.ingestor_states.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_ingestor_health(self):
        async def update(health: IngestorHealth):
            await health.save()
            self._report_health(health, self._loggers.ingestor_health)

        self._subscriptions.append(
            self.rmf.ingestor_health.subscribe(lambda x: self._create_task(update(x)))
        )
