import asyncio
from typing import Optional

from rx.operators import flat_map, group_by
from rx.operators import map as rx_map
from rx.scheduler.scheduler import Scheduler

from ..models import DoorHealth, HealthStatus
from .gateway import RmfGateway
from .operators import heartbeat


class HealthWatchdog:
    LIVELINESS = 10

    def __init__(
        self, rmf_gateway: RmfGateway, *, scheduler: Optional[Scheduler] = None
    ):
        self.rmf = rmf_gateway

        def map_door_health(has_heartbeat: bool, door_name: str) -> DoorHealth:
            if has_heartbeat:
                return DoorHealth(name=door_name, health_status=HealthStatus.HEALTHY)
            return DoorHealth(name=door_name, health_status=HealthStatus.DEAD)

        self.rmf.door_states.pipe(
            group_by(lambda x: x.door_name),
            flat_map(
                lambda x: x.pipe(
                    heartbeat(self.LIVELINESS),
                    rx_map(lambda y: map_door_health(y, x.key)),
                )
            ),
        ).subscribe(
            lambda x: asyncio.get_event_loop().call_soon_threadsafe(
                lambda: self.rmf.door_health.on_next(x)
            ),
            scheduler=scheduler,
        )
