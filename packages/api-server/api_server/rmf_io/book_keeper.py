import asyncio
import logging
from typing import Coroutine

from reactivex.abc import DisposableBase
from reactivex.subject import Subject

from api_server.models import (
    BeaconState,
    BuildingMap,
    DispenserState,
    DoorState,
    IngestorState,
    LiftState,
)

from .events import RmfEvents


class RmfBookKeeperEvents:
    def __init__(self):
        self.task_summary_written = Subject()


class RmfBookKeeper:
    def __init__(
        self,
        rmf_events: RmfEvents,
    ):
        self.rmf = rmf_events
        self.bookkeeper_events = RmfBookKeeperEvents()
        self._loop: asyncio.AbstractEventLoop
        self._pending_tasks = set()
        self._subscriptions: list[DisposableBase] = []

    async def start(self):
        self._loop = asyncio.get_event_loop()
        self._record_beacon_state()
        self._record_building_map()
        self._record_door_state()
        self._record_lift_state()
        self._record_dispenser_state()
        self._record_ingestor_state()

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

    def _record_beacon_state(self):
        async def update(beacon_state: BeaconState):
            await beacon_state.save()
            logging.debug(beacon_state.model_dump_json())

        self._subscriptions.append(
            self.rmf.beacons.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_building_map(self):
        async def update(building_map: BuildingMap):
            if not building_map:
                return
            await building_map.save()
            logging.debug(building_map.model_dump_json())

        self._subscriptions.append(
            self.rmf.building_map.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_door_state(self):
        async def update(door_state: DoorState):
            await door_state.save()
            logging.debug(door_state.model_dump_json())

        self._subscriptions.append(
            self.rmf.door_states.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_lift_state(self):
        async def update(lift_state: LiftState):
            await lift_state.save()
            logging.debug(lift_state.model_dump_json())

        self._subscriptions.append(
            self.rmf.lift_states.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_dispenser_state(self):
        async def update(dispenser_state: DispenserState):
            await dispenser_state.save()
            logging.debug(dispenser_state.model_dump_json())

        self._subscriptions.append(
            self.rmf.dispenser_states.subscribe(lambda x: self._create_task(update(x)))
        )

    def _record_ingestor_state(self):
        async def update(ingestor_state: IngestorState):
            await ingestor_state.save()
            logging.debug(ingestor_state.model_dump_json())

        self._subscriptions.append(
            self.rmf.ingestor_states.subscribe(lambda x: self._create_task(update(x)))
        )
