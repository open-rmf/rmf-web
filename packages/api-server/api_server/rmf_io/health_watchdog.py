import json
import logging
from typing import Any, Callable, Optional

from rx import Observable
from rx.operators import flat_map, group_by
from rx.operators import map as rx_map
from rx.scheduler.scheduler import Scheduler

from ..models import BasicHealthModel, DoorHealth, HealthStatus, LiftHealth
from .gateway import RmfGateway
from .operators import heartbeat


class HealthWatchdog:
    LIVELINESS = 10

    def __init__(
        self,
        rmf_gateway: RmfGateway,
        *,
        scheduler: Optional[Scheduler] = None,
        logger: logging.Logger = None,
    ):
        self.rmf = rmf_gateway
        self.scheduler = scheduler
        self.logger = logger or logging.getLogger(self.__class__.__name__)

        self._watch_door_health()
        self._watch_lift_health()

    def _watch_heartbeat(
        self,
        HealthModel: BasicHealthModel,
        source: Observable,
        target: Observable,
        key_mapper: Callable[[Any], str],
    ):
        def map_health(has_heartbeat: bool, name: str):
            if has_heartbeat:
                return HealthModel(name=name, health_status=HealthStatus.HEALTHY)
            return HealthModel(
                name=name,
                health_status=HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        def report_health(health: BasicHealthModel):
            message = json.dumps(
                {
                    "name": health.name,
                    "health_status": str(health.health_status),
                    "health_message": health.health_message,
                }
            )
            if health.health_status == HealthStatus.UNHEALTHY:
                self.logger.warning(message)
            elif health.health_status == HealthStatus.DEAD:
                self.logger.error(message)
            else:
                self.logger.info(message)
            target.on_next(health)

        source.pipe(
            group_by(key_mapper),
            flat_map(
                lambda x: x.pipe(
                    heartbeat(self.LIVELINESS),
                    rx_map(lambda y: map_health(y, x.key)),
                )
            ),
        ).subscribe(report_health, scheduler=self.scheduler)

    def _watch_door_health(self):
        self._watch_heartbeat(
            DoorHealth,
            self.rmf.door_states,
            self.rmf.door_health,
            lambda x: x.door_name,
        )

    def _watch_lift_health(self):
        self._watch_heartbeat(
            LiftHealth,
            self.rmf.lift_states,
            self.rmf.lift_health,
            lambda x: x.lift_name,
        )
