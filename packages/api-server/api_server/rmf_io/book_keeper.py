import asyncio
import logging
from datetime import timedelta
from typing import Any, Callable, Optional, Union

import rx
from rx import Observable
from rx.operators import sample, group_by, flat_map, Mapper
from rx.scheduler.eventloop import AsyncIOScheduler
from rx.scheduler.scheduler import Scheduler

from .gateway import RmfGateway
from ..repositories import SqlRepository


def grouped_sample(
    key_mapper: Mapper,
    sampler: Union[timedelta, float, Observable],
):
    '''
    Combination of "group_by", "flat_map" and "sample", groups an observable sequence by the
    "key_mapper" function, maps the resulting observable sequences with the "sample" operator
    and flatten it into a single observable sequence.
    '''
    return rx.pipe(
        group_by(key_mapper),
        flat_map(lambda x: x.pipe(sample(sampler))),
    )


class RmfBookKeeper():
    FrequencyStates = 1
    FrequencyRobotPlans = 5

    def __init__(
        self,
        rmf_gateway: RmfGateway,
        repo: SqlRepository,
        logger: logging.Logger = None,
    ):
        self.rmf = rmf_gateway
        self.repo = repo
        self.logger = logger or logging.getLogger(self.__class__.__name__)

    def start(
        self,
        loop: asyncio.AbstractEventLoop = None,
        scheduler: Optional[Scheduler] = None,
    ):
        loop = loop or asyncio.get_event_loop()

        self.rmf.door_states.pipe(
            grouped_sample(lambda x: x.door_name, RmfBookKeeper.FrequencyStates),
        ).subscribe(
            lambda x: loop.create_task(self.repo.write_door_state(x)),
            scheduler=scheduler,
        )
