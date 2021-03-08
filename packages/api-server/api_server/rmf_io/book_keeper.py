import asyncio
import json
import logging
from datetime import timedelta
from typing import Optional, Union

import rx
from building_map_msgs.msg import BuildingMap, Level
from rosidl_runtime_py.convert import message_to_ordereddict
from rx import Observable
from rx.operators import Mapper, flat_map, group_by, sample
from rx.scheduler.scheduler import Scheduler

from ..repositories import SqlRepository
from .gateway import RmfGateway


def grouped_sample(
    key_mapper: Mapper,
    sampler: Union[timedelta, float, Observable],
):
    """
    Combination of "group_by", "flat_map" and "sample", groups an observable sequence by the
    "key_mapper" function, maps the resulting observable sequences with the "sample" operator
    and flatten it into a single observable sequence.
    """
    return rx.pipe(
        group_by(key_mapper),
        flat_map(lambda x: x.pipe(sample(sampler))),
    )


class RmfBookKeeper:
    FrequencyStates = 1
    FrequencyRobotPlans = 5

    def __init__(
        self,
        rmf_gateway: RmfGateway,
        repo: SqlRepository,
        loop: asyncio.AbstractEventLoop = None,
        scheduler: Optional[Scheduler] = None,
        logger: logging.Logger = None,
    ):
        self.rmf = rmf_gateway
        self.repo = repo
        self.loop: asyncio.AbstractEventLoop = loop or asyncio.get_event_loop()
        self.scheduler = scheduler or None
        self.logger = logger or logging.getLogger(self.__class__.__name__)

    def start(
        self,
    ):
        async def update_door_state(door_state):
            await self.repo.update_door_state(door_state)
            self.logger.info(json.dumps(message_to_ordereddict(door_state)))

        self.rmf.door_states.pipe(
            grouped_sample(lambda x: x.door_name, RmfBookKeeper.FrequencyStates),
        ).subscribe(
            lambda x: self.loop.create_task(update_door_state(x)),
            scheduler=self.scheduler,
        )

        async def update_building_map(building_map: Optional[BuildingMap]):
            if not building_map:
                return
            all_doors = []
            for level in building_map.levels:
                level: Level
                all_doors.extend(level.doors)
            await self.repo.sync_doors(all_doors)

        self.rmf.building_map.subscribe(
            lambda x: self.loop.create_task(update_building_map(x)),
            scheduler=self.scheduler,
        )

        self._watch_door_health()
        self._watch_lift_health()

    def _watch_door_health(self):
        async def on_next(door_health):
            await self.repo.update_door_health(door_health)

        self.rmf.door_health.subscribe(
            lambda x: self.loop.create_task(on_next(x)), scheduler=self.scheduler
        )

    def _watch_lift_health(self):
        async def on_next(lift_health):
            await self.repo.update_lift_health(lift_health)

        self.rmf.lift_health.subscribe(
            lambda x: self.loop.create_task(on_next(x)), scheduler=self.scheduler
        )
