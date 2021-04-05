import asyncio
import json
import logging
from collections import namedtuple
from typing import List

from rmf_dispenser_msgs.msg import DispenserState
from rmf_door_msgs.msg import DoorState
from rmf_fleet_msgs.msg import FleetState
from rmf_ingestor_msgs.msg import IngestorState
from rmf_lift_msgs.msg import LiftState
from rmf_task_msgs.msg import Tasks
from rosidl_runtime_py.convert import message_to_ordereddict
from rx.core.typing import Disposable

from ..models import tortoise_models as ttm
from .gateway import RmfGateway


class RmfBookKeeper:
    _ChildLoggers = namedtuple(
        "_ChildLoggers",
        [
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
        rmf_gateway: RmfGateway,
        *,
        loop: asyncio.AbstractEventLoop = None,
        logger: logging.Logger = None,
    ):
        self.rmf = rmf_gateway
        self.loop: asyncio.AbstractEventLoop = loop or asyncio.get_event_loop()
        self._main_logger = logger or logging.getLogger(self.__class__.__name__)

        self._loggers = self._ChildLoggers(
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
        self._started = False

    def start(self):
        if self._started:
            return
        self._record_door_state()
        self._record_door_health()
        self._record_lift_state()
        self._record_lift_health()
        self._record_dispenser_state()
        self._record_dispenser_health()
        self._record_ingestor_state()
        self._record_ingestor_health()
        self._record_fleet_state()
        self._record_robot_health()
        self._record_task_summary()
        self._started = True

    def stop(self):
        for sub in self._subscriptions:
            sub.dispose()
        self._subscriptions.clear()
        self._started = False

    @staticmethod
    def _report_health(health: ttm.BasicHealthModel, logger: logging.Logger):
        message = json.dumps(
            {
                "id": health.id_,
                "health_status": str(health.health_status),
                "health_message": health.health_message,
            }
        )
        if health.health_status == ttm.HealthStatus.UNHEALTHY:
            logger.warning(message)
        elif health.health_status == ttm.HealthStatus.DEAD:
            logger.error(message)
        else:
            logger.info(message)

    def _record_door_state(self):
        async def update(door_state: DoorState):
            await ttm.DoorState.update_or_create_from_rmf(door_state)
            self._loggers.door_state.info(
                json.dumps(message_to_ordereddict(door_state))
            )

        self._subscriptions.append(
            self.rmf.door_states.subscribe(lambda x: self.loop.create_task(update(x)))
        )

    def _record_door_health(self):
        async def update(health: ttm.DoorHealth):
            await ttm.DoorHealth.update_or_create(
                {
                    "health_status": health.health_status,
                    "health_message": health.health_message,
                },
                id_=health.id_,
            )
            self._report_health(health, self._loggers.door_health)

        self._subscriptions.append(
            self.rmf.door_health.subscribe(lambda x: self.loop.create_task(update(x)))
        )

    def _record_lift_state(self):
        async def update(lift_state: LiftState):
            await ttm.LiftState.update_or_create_from_rmf(lift_state)
            self._loggers.lift_state.info(
                json.dumps(message_to_ordereddict(lift_state))
            )

        self._subscriptions.append(
            self.rmf.lift_states.subscribe(lambda x: self.loop.create_task(update(x)))
        )

    def _record_lift_health(self):
        async def update(health: ttm.LiftHealth):
            await ttm.LiftHealth.update_or_create(
                {
                    "health_status": health.health_status,
                    "health_message": health.health_message,
                },
                id_=health.id_,
            )
            self._report_health(health, self._loggers.lift_health)

        self._subscriptions.append(
            self.rmf.lift_health.subscribe(lambda x: self.loop.create_task(update(x)))
        )

    def _record_dispenser_state(self):
        async def update(dispenser_state: DispenserState):
            await ttm.DispenserState.update_or_create_from_rmf(dispenser_state)
            self._loggers.dispenser_state.info(
                json.dumps(message_to_ordereddict(dispenser_state))
            )

        self._subscriptions.append(
            self.rmf.dispenser_states.subscribe(
                lambda x: self.loop.create_task(update(x))
            )
        )

    def _record_dispenser_health(self):
        async def update(health: ttm.DispenserHealth):
            await ttm.DispenserHealth.update_or_create(
                {
                    "health_status": health.health_status,
                    "health_message": health.health_message,
                },
                id_=health.id_,
            )
            self._report_health(health, self._loggers.dispenser_health)

        self._subscriptions.append(
            self.rmf.dispenser_health.subscribe(
                lambda x: self.loop.create_task(update(x))
            )
        )

    def _record_ingestor_state(self):
        async def update(ingestor_state: IngestorState):
            await ttm.IngestorState.update_or_create_from_rmf(ingestor_state)
            self._loggers.ingestor_state.info(
                json.dumps(message_to_ordereddict(ingestor_state))
            )

        self._subscriptions.append(
            self.rmf.ingestor_states.subscribe(
                lambda x: self.loop.create_task(update(x))
            )
        )

    def _record_ingestor_health(self):
        async def update(health: ttm.IngestorHealth):
            await ttm.IngestorHealth.update_or_create(
                {
                    "health_status": health.health_status,
                    "health_message": health.health_message,
                },
                id_=health.id_,
            )
            self._report_health(health, self._loggers.ingestor_health)

        self._subscriptions.append(
            self.rmf.ingestor_health.subscribe(
                lambda x: self.loop.create_task(update(x))
            )
        )

    def _record_fleet_state(self):
        async def update(fleet_state: FleetState):
            await ttm.FleetState.update_or_create_from_rmf(fleet_state)
            self._loggers.fleet_state.info(
                json.dumps(message_to_ordereddict(fleet_state))
            )

        self._subscriptions.append(
            self.rmf.fleet_states.subscribe(lambda x: self.loop.create_task(update(x)))
        )

    def _record_robot_health(self):
        async def update(health: ttm.RobotHealth):
            await ttm.RobotHealth.update_or_create(
                {
                    "health_status": health.health_status,
                    "health_message": health.health_message,
                },
                id_=health.id_,
            )
            self._report_health(health, self._loggers.robot_health)

        self._subscriptions.append(
            self.rmf.robot_health.subscribe(lambda x: self.loop.create_task(update(x)))
        )

    def _record_task_summary(self):
        async def update(tasks: Tasks):
            for task in tasks.tasks:
                await ttm.TaskSummary.update_or_create_from_rmf(task)
                self._loggers.task_summary.info(
                    json.dumps(message_to_ordereddict(task))
                )

        self._subscriptions.append(
            self.rmf.task_summaries.subscribe(
                lambda x: self.loop.create_task(update(x))
            )
        )
