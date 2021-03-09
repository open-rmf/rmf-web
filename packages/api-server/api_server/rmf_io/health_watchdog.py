import json
import logging
from typing import Any, Callable, Optional

from rmf_door_msgs.msg import DoorMode, DoorState
from rx import Observable
from rx import operators as op
from rx.scheduler.scheduler import Scheduler

from ..models import BasicHealthModel, DoorHealth, HealthStatus, LiftHealth
from .gateway import RmfGateway
from .operators import heartbeat, most_critical


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

    def _report_health(self, target: Observable):
        """
        :param target: Observable[BasicHealthModel]
        """

        def on_next(health: BasicHealthModel):
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

        return on_next

    def _watch_heartbeat(
        self,
        HealthModel: BasicHealthModel,
        source: Observable,
        key_mapper: Callable[[Any], str],
    ) -> Observable:
        """
        :returns: Observable[BasicHealthModel]
        """

        def map_health(has_heartbeat: bool, name: str):
            if has_heartbeat:
                return HealthModel(name=name, health_status=HealthStatus.HEALTHY)
            return HealthModel(
                name=name,
                health_status=HealthStatus.DEAD,
                health_message="heartbeat failed",
            )

        return source.pipe(
            op.group_by(key_mapper),
            op.flat_map(
                lambda x: x.pipe(
                    heartbeat(self.LIVELINESS),
                    op.map(lambda y: map_health(y, x.key)),
                )
            ),
        )

    def _watch_door_health(self):
        def door_mode_to_health(state: DoorState):
            if state.current_mode.value == DoorMode.MODE_OFFLINE:
                return DoorHealth(
                    name=state.door_name,
                    health_status=HealthStatus.UNHEALTHY,
                    health_message="offline",
                )
            if state.current_mode.value == DoorMode.MODE_UNKNOWN:
                return DoorHealth(
                    name=state.door_name,
                    health_status=HealthStatus.UNHEALTHY,
                    health_message="unknown",
                )
            return DoorHealth(
                name=state.door_name,
                health_status=HealthStatus.HEALTHY,
            )

        door_mode_health = self.rmf.door_states.pipe(
            op.map(door_mode_to_health),
            op.timestamp(),
        )

        heartbeat_health = self._watch_heartbeat(
            DoorHealth,
            self.rmf.door_states,
            lambda x: x.door_name,
        ).pipe(op.timestamp())

        heartbeat_health.pipe(
            op.combine_latest(door_mode_health),
            most_critical(),
        ).subscribe(self._report_health(self.rmf.door_health), scheduler=self.scheduler)

    def _watch_lift_health(self):
        self._watch_heartbeat(
            LiftHealth,
            self.rmf.lift_states,
            lambda x: x.lift_name,
        ).subscribe(self._report_health(self.rmf.lift_health), scheduler=self.scheduler)
