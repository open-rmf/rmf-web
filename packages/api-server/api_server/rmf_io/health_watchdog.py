import json
import logging
from typing import Any, Callable, Optional, Sequence

import rx
from rmf_door_msgs.msg import DoorMode, DoorState
from rx import Observable
from rx.core.operators.timestamp import Timestamp
from rx.operators import combine_latest, flat_map, group_by
from rx.operators import map as rx_map
from rx.operators import timestamp
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
            group_by(key_mapper),
            flat_map(
                lambda x: x.pipe(
                    heartbeat(self.LIVELINESS),
                    rx_map(lambda y: map_health(y, x.key)),
                )
            ),
        )

    @staticmethod
    def _most_critical():
        """
        Maps an observable sequence of a sequence of timestamp of BasicHealthModel to an
        observable sequence of BasicHealthModel with the most critical health status. If there
        are multiple BasicHealthModel with the same criticality, the most recent item is
        chosen.
        """

        def criticality(health_status: HealthStatus):
            """
            Converts a health status into an int such that less healthy > more healthy.
            """
            if health_status == HealthStatus.HEALTHY:
                return 0
            if health_status == HealthStatus.UNHEALTHY:
                return 1
            if health_status == HealthStatus.DEAD:
                return 2
            raise Exception("unknown health status")

        def get_most_critical(health_statuses: Sequence[Timestamp]):
            """
            :param health_status: Sequence[Timestamp[HealthStatus]]
            """
            most_crit = health_statuses[0]
            for health in health_statuses:
                cur = criticality(most_crit.value.health_status)
                other = criticality(health.value.health_status)
                if other > cur:
                    most_crit = health
                elif other == cur:
                    if health.timestamp > most_crit.timestamp:
                        most_crit = health
            return most_crit.value

        return rx.pipe(rx_map(get_most_critical))

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
            rx_map(door_mode_to_health),
            timestamp(),
        )

        heartbeat_health = self._watch_heartbeat(
            DoorHealth,
            self.rmf.door_states,
            lambda x: x.door_name,
        ).pipe(timestamp())

        heartbeat_health.pipe(
            combine_latest(door_mode_health),
            self._most_critical(),
        ).subscribe(self._report_health(self.rmf.door_health), scheduler=self.scheduler)

    def _watch_lift_health(self):
        self._watch_heartbeat(
            LiftHealth,
            self.rmf.lift_states,
            lambda x: x.lift_name,
        ).subscribe(self._report_health(self.rmf.lift_health), scheduler=self.scheduler)
