import asyncio
import json
import logging
from typing import Coroutine, List

from rx.core.typing import Disposable
from rx.subject.subject import Subject

from api_server.models import (
    BeaconState,
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
    def __init__(
        self,
        rmf_events: RmfEvents,
    ):
        self.rmf = rmf_events
        self.bookkeeper_events = RmfBookKeeperEvents()
        self._loop: asyncio.AbstractEventLoop
        self._pending_tasks = set()
        self._subscriptions: List[Disposable] = []

    async def start(self):
        self._loop = asyncio.get_event_loop()
        self._record_beacon_state()
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
    def _report_health(health: BaseBasicHealth):
        message = health.json()
        if health.health_status == HealthStatus.UNHEALTHY:
            logging.warning(message)
        elif health.health_status == HealthStatus.DEAD:
            logging.error(message)
        else:
            logging.info(message)

    def _record_beacon_state(self):
        async def update(beacon_state: BeaconState):
            await beacon_state.save()
            logging.debug(json.dumps(beacon_state.dict()))

        self._subscriptions.append(
            self.rmf.beacons.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_building_map(self):
        async def update(building_map: BuildingMap):
            if not building_map:
                return
            await building_map.save()
            logging.debug(json.dumps(building_map.dict()))

        self._subscriptions.append(
            self.rmf.building_map.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_door_state(self):
        async def update(door_state: DoorState):
            await door_state.save()
            logging.debug(json.dumps(door_state.dict()))

        self._subscriptions.append(
            self.rmf.door_states.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_door_health(self):
        async def update(health: DoorHealth):
            await health.save()
            self._report_health(health)

        self._subscriptions.append(
            self.rmf.door_health.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_lift_state(self):
        async def update(lift_state: LiftState):
            await lift_state.save()
            logging.debug(lift_state.json())

        self._subscriptions.append(
            self.rmf.lift_states.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_lift_health(self):
        async def update(health: LiftHealth):
            await health.save()
            self._report_health(health)

        self._subscriptions.append(
            self.rmf.lift_health.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_dispenser_state(self):
        async def update(dispenser_state: DispenserState):
            await dispenser_state.save()
            logging.debug(dispenser_state.json())

        self._subscriptions.append(
            self.rmf.dispenser_states.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_dispenser_health(self):
        async def update(health: DispenserHealth):
            await health.save()
            self._report_health(health)

        self._subscriptions.append(
            self.rmf.dispenser_health.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_ingestor_state(self):
        async def update(ingestor_state: IngestorState):
            await ingestor_state.save()
            logging.debug(ingestor_state.json())

        self._subscriptions.append(
            self.rmf.ingestor_states.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_ingestor_health(self):
        async def update(health: IngestorHealth):
            await health.save()
            self._report_health(health)

        self._subscriptions.append(
            self.rmf.ingestor_health.subscribe(lambda x: self._create_task(update(x)))
        )
